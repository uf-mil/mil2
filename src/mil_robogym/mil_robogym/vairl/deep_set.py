import numbers

import numpy as np
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

    def forward(self, x: np.ndarray) -> torch.Tensor:
        """
        Forward function for deepset class.

        x: array of a single unordered set or batch of unordered sets

        b: batch size
        m: number of objects in each unordered set
        n: feature size of object determined by convert_object_to_tensor
        """
        # Handle completely empty input early
        if len(x) == 0:
            # Treat as single empty set: [m=0, n=input_dim]
            pooled = torch.zeros(self.nested_network[-2].out_features)
            out = self.outer_network(pooled)
            return out.view(self.output_shape)

        # Convert raw input into tensor
        x_transformed = [self.convert_object_to_tensor(obj) for obj in x]

        # Stack into a single tensor
        if isinstance(x_transformed[0], torch.Tensor):
            x = torch.stack(x_transformed).float()
        else:
            x = torch.from_numpy(np.array(x_transformed)).float()

        # Single input
        if x.dim() == 2:  # [m, n]
            if x.size(-1) != self.input_dim:
                raise ValueError(
                    f"Object feature dimension '{x.size(-1)}' does not match expected input_dim '{self.input_dim}'",
                )
            embedded = self.nested_network(x)  # [m, embed_dim]
            pooled = embedded.sum(dim=0)  # [embed_dim]
            out = self.outer_network(pooled)  # [output_dim]
            return out.view(self.output_shape)

        # Batched inputs
        if x.dim() == 3:  # [b, m, n]
            if x.size(-1) != self.input_dim:
                raise ValueError(
                    f"Object feature dimension '{x.size(-1)}' does not match expected input_dim '{self.input_dim}'",
                )
            embedded = self.nested_network(x)  # [b, m, embed_dim]
            pooled = embedded.sum(dim=1)  # [b, embed_dim]
            out = self.outer_network(pooled)  # [b, output_dim]
            return out.view(x.size(0), *self.output_shape)

        raise ValueError(
            f"forward expects x with shape [m, n] or [b, m, n], instead got dimension: {x.dim()} for tensor of shape: {x.shape}",
        )

    def convert_object_to_tensor(self, obj: any) -> torch.Tensor:
        """
        obj
        """
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
