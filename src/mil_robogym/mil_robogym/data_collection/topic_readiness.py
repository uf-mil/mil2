from __future__ import annotations

from dataclasses import dataclass

from .ros_graph import query_topics_in_graph
from .sample_input_topics import collect_resolved_topic_payloads_once
from .types import RoboGymProjectYaml

DEFAULT_TOPIC_READINESS_TIMEOUT_S = 5.0


@dataclass(frozen=True)
class TopicReadinessReport:
    requested_topics: list[str]
    resolved_topics: dict[str, str]
    missing_topics: list[str]
    type_unresolved_topics: list[str]
    silent_topics: list[str]

    @property
    def ready(self) -> bool:
        return (
            not self.missing_topics
            and not self.type_unresolved_topics
            and not self.silent_topics
        )


class TopicReadinessError(RuntimeError):
    def __init__(
        self,
        report: TopicReadinessReport,
        *,
        operation: str,
        topic_label: str = "required topics",
    ) -> None:
        self.report = report
        self.operation = operation
        self.topic_label = topic_label
        super().__init__(
            _format_topic_readiness_failure(
                report,
                operation=operation,
                topic_label=topic_label,
            ),
        )


def _dedupe_preserve_order(values: list[str]) -> list[str]:
    return list(dict.fromkeys(values))


def _format_topic_readiness_failure(
    report: TopicReadinessReport,
    *,
    operation: str,
    topic_label: str,
) -> str:
    lines = [f"Cannot {operation} because {topic_label} are not ready."]
    if report.missing_topics:
        lines.append(f"Missing from ROS 2 graph: {report.missing_topics}")
    if report.type_unresolved_topics:
        lines.append(
            "Missing ROS 2 type information for: " f"{report.type_unresolved_topics}",
        )
    if report.silent_topics:
        lines.append(
            "Not receiving messages within the readiness timeout: "
            f"{report.silent_topics}",
        )
    return "\n".join(lines)


def check_topics_ready(
    topics: list[str],
    *,
    timeout_s: float = DEFAULT_TOPIC_READINESS_TIMEOUT_S,
    start_simulation: bool = False,
) -> TopicReadinessReport:
    """
    Check whether topics exist in the ROS 2 graph and are actively publishing.
    """
    requested_topics = _dedupe_preserve_order(list(topics))
    if not requested_topics:
        return TopicReadinessReport(
            requested_topics=[],
            resolved_topics={},
            missing_topics=[],
            type_unresolved_topics=[],
            silent_topics=[],
        )

    simulation_client = None
    if start_simulation:
        from mil_robogym.clients.world_control_client import WorldControlClient

        simulation_client = WorldControlClient()
        simulation_client.acquire_simulation_hold()

    try:
        resolution = query_topics_in_graph(requested_topics, timeout_s=timeout_s)

        resolved_topic_names = [
            resolution.resolved_topics[topic][0]
            for topic in requested_topics
            if topic in resolution.resolved_topics
        ]
        resolved_message_types = {
            resolved_topic: topic_types[0]
            for resolved_topic, topic_types in resolution.resolved_topics.values()
        }

        _received_messages, silent_resolved_topics = (
            collect_resolved_topic_payloads_once(
                resolved_topic_names,
                timeout_s=timeout_s,
                topic_message_types=resolved_message_types,
            )
        )
        silent_resolved = set(silent_resolved_topics)
        silent_topics = [
            topic
            for topic in requested_topics
            if topic in resolution.resolved_topics
            and resolution.resolved_topics[topic][0] in silent_resolved
        ]

        return TopicReadinessReport(
            requested_topics=requested_topics,
            resolved_topics={
                topic: resolution.resolved_topics[topic][0]
                for topic in requested_topics
                if topic in resolution.resolved_topics
            },
            missing_topics=resolution.missing_topics,
            type_unresolved_topics=resolution.type_unresolved_topics,
            silent_topics=silent_topics,
        )
    finally:
        if simulation_client is not None:
            simulation_client.release_simulation_hold()


def ensure_topics_ready(
    topics: list[str],
    *,
    timeout_s: float = DEFAULT_TOPIC_READINESS_TIMEOUT_S,
    operation: str,
    topic_label: str = "required topics",
    start_simulation: bool = False,
) -> TopicReadinessReport:
    """
    Raise a descriptive error when required topics are not ready.
    """
    report = check_topics_ready(
        topics,
        timeout_s=timeout_s,
        start_simulation=start_simulation,
    )
    if not report.ready:
        raise TopicReadinessError(
            report,
            operation=operation,
            topic_label=topic_label,
        )
    return report


def project_input_topics(project: RoboGymProjectYaml) -> list[str]:
    return list(project["input_topics"].keys())


def ensure_project_input_topics_ready(
    project: RoboGymProjectYaml,
    *,
    timeout_s: float = DEFAULT_TOPIC_READINESS_TIMEOUT_S,
    operation: str,
    start_simulation: bool = False,
) -> TopicReadinessReport:
    return ensure_topics_ready(
        project_input_topics(project),
        timeout_s=timeout_s,
        operation=operation,
        topic_label="project input topics",
        start_simulation=start_simulation,
    )
