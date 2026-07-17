import numbers

import torch
import torch.nn as nn


class DeepSet(nn.Module):
    def __init__(self, obj: any, output_shape: int):
        """
        Init function for deepset class

        obj: object to be convert into a tensor
        output_shape: final output_shape of deepset model
        """
        super().__init__()

        # Validate output_shape
        if isinstance(output_shape, int):
            self.output_shape = (output_shape,)
            self.output_dim = output_shape
        else:
            raise TypeError("output_shape must be an int")

        # Convert object to tensor and get shape
        flattened = self.convert_object_to_tensor(obj)
        if flattened.numel() == 0:
            raise ValueError("Input object flattens to an empty tensor")
        self.reference_input_shape = tuple(flattened.shape)
        self.input_dim = int(flattened.numel())

        # Set hyperparameters
        hidden_dim = 32
        embed_dim = 32

        # Inner network phi: applied to the flattened object
        self.nested_network = nn.Sequential(
            nn.Linear(self.input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, embed_dim),
            nn.ReLU(),
        )

        # Outer network rho: maps pooled embedding to requested output shape.
        self.outer_network = nn.Sequential(
            nn.Linear(embed_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, self.output_dim),
        )

    def forward(self, x: list | torch.Tensor) -> torch.Tensor:
        """
        Forward function for deepset class.

        Supported input layouts:
          - ``[m, n]`` tensor for a single set of ``m`` objects
          - ``[b, m, n]`` tensor for a batch of ``b`` sets
          - raw Python objects representing a single set or batch of sets

        b: batch size
        m: number of objects in each unordered set
        n: feature size of object determined by convert_object_to_tensor
        """
        target_device = self._target_device()

        if isinstance(x, torch.Tensor):
            return self._forward_tensor_input(
                x.to(device=target_device, dtype=torch.float32),
            )

        if len(x) == 0:
            return self._forward_empty_input(target_device)

        is_batch = isinstance(x[0], (list, tuple))

        if not is_batch:
            x_transformed = [
                self.convert_object_to_tensor(obj).to(device=target_device) for obj in x
            ]
            x = torch.stack(x_transformed).to(
                device=target_device,
                dtype=torch.float32,
            )
            return self._forward_tensor_input(x)

        batch_tensors = []

        for subset in x:
            if len(subset) == 0:
                empty = torch.zeros(self.input_dim, device=target_device)
                batch_tensors.append(empty.unsqueeze(0))
                continue

            transformed = [
                self.convert_object_to_tensor(obj).to(device=target_device)
                for obj in subset
            ]
            subset_tensor = torch.stack(transformed).to(
                device=target_device,
                dtype=torch.float32,
            )
            batch_tensors.append(subset_tensor)

        max_m = max(t.size(0) for t in batch_tensors)

        padded_batch = []
        for tensor in batch_tensors:
            if tensor.size(0) < max_m:
                pad = torch.zeros(
                    max_m - tensor.size(0),
                    self.input_dim,
                    device=target_device,
                )
                tensor = torch.cat([tensor, pad], dim=0)
            padded_batch.append(tensor)

        x = torch.stack(padded_batch).to(device=target_device, dtype=torch.float32)
        return self._forward_tensor_input(x)

    def _forward_empty_input(self, device: torch.device) -> torch.Tensor:
        pooled = torch.zeros(
            self.nested_network[-2].out_features,
            device=device,
        )
        out = self.outer_network(pooled)
        return out.view(self.output_shape)

    def _forward_tensor_input(self, x: torch.Tensor) -> torch.Tensor:
        if x.ndim not in (2, 3):
            raise ValueError("x must have shape [m, n] or [b, m, n].")

        if x.size(-1) != self.input_dim:
            raise ValueError(
                f"Object feature dimension '{x.size(-1)}' does not match expected input_dim '{self.input_dim}'",
            )

        if x.ndim == 2:
            embedded = self.nested_network(x)
            pooled = embedded.sum(dim=0)
            out = self.outer_network(pooled)
            return out.view(self.output_shape)

        embedded = self.nested_network(x)
        pooled = embedded.sum(dim=1)
        out = self.outer_network(pooled)
        return out.view(x.size(0), *self.output_shape)

    def _target_device(self) -> torch.device:
        return next(self.parameters()).device

    def convert_object_to_tensor(self, obj: any) -> torch.Tensor:
        """Convert a nested Python object into a flat numeric tensor."""
        flattened_values = []
        stack = [obj]
        seen_ids = set()

        while stack:
            curr_obj = stack.pop()

            # Check non reference objects
            if curr_obj is None:
                continue

            if isinstance(curr_obj, bool):
                flattened_values.append(float(curr_obj))
                continue

            if isinstance(curr_obj, numbers.Real):
                flattened_values.append(float(curr_obj))
                continue

            if isinstance(curr_obj, str):
                # Text metadata (for example YOLO class labels) is excluded from the
                # numeric embedding space instead of failing preprocessing.
                continue

            # Object id for reference objects, check if seen before to skip
            obj_id = id(curr_obj)
            if obj_id in seen_ids:
                continue

            # Check reference objects
            if isinstance(curr_obj, (list, tuple)):
                seen_ids.add(obj_id)
                for item in reversed(curr_obj):
                    stack.append(item)
                continue

            if isinstance(curr_obj, dict):
                seen_ids.add(obj_id)
                values = list(curr_obj.values())
                for value in reversed(values):
                    stack.append(value)
                continue

            if isinstance(curr_obj, set):
                seen_ids.add(obj_id)
                ordered_items = sorted(curr_obj, key=repr)
                for item in reversed(ordered_items):
                    stack.append(item)
                continue

            if hasattr(curr_obj, "__dict__") or hasattr(curr_obj, "__slots__"):
                seen_ids.add(obj_id)

                if hasattr(curr_obj, "__dict__"):
                    attrs = list(vars(curr_obj).items())
                    for _, value in reversed(attrs):
                        stack.append(value)

                if hasattr(curr_obj, "__slots__"):
                    slot_names = curr_obj.__slots__
                    if isinstance(slot_names, str):
                        slot_names = [slot_names]
                    for slot_name in reversed(list(slot_names)):
                        if hasattr(curr_obj, slot_name):
                            stack.append(getattr(curr_obj, slot_name))
                continue

            raise TypeError(
                f"Unsupported type encountered: {type(curr_obj).__name__}. "
                "Supported types: int, float, list, tuple, dict, set, and custom objects.",
            )

        return torch.tensor(flattened_values, dtype=torch.float32)
