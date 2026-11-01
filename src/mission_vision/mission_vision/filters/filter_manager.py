
import importlib


class FilterManager:
    """
    Dynamically loads filters listed in the mission YAML.

    To add a new filter:
      1. Put a .py file in mission_vision/filters/
      2. Implement class __init__(**params) and process(data, context)
      3. Add it to the desired YAML stage

    vision_node.py does not need to be edited.
    """

    VALID_STAGES = ("preprocess", "detection", "result")

    def __init__(self, pipeline_config, logger=None):
        self.logger = logger
        self.filters = {
            stage: []
            for stage in self.VALID_STAGES
        }

        pipeline_config = pipeline_config or {}

        for stage in self.VALID_STAGES:
            entries = pipeline_config.get(stage, []) or []

            if not isinstance(entries, list):
                raise ValueError(
                    f"pipeline.{stage} must be a YAML list"
                )

            for entry in entries:
                self._load_filter(stage, entry)

    def _load_filter(self, stage, entry):
        if not isinstance(entry, dict):
            raise ValueError(
                f"Each pipeline.{stage} entry must be a mapping"
            )

        if not bool(entry.get("enabled", True)):
            return

        module_name = entry.get("module")
        class_name = entry.get("class")
        params = entry.get("params", {}) or {}

        if not module_name or not class_name:
            raise ValueError(
                f"pipeline.{stage} entries require "
                "'module' and 'class'"
            )

        module = importlib.import_module(str(module_name))
        filter_class = getattr(module, str(class_name))
        filter_object = filter_class(**params)

        if not hasattr(filter_object, "process"):
            raise TypeError(
                f"{module_name}.{class_name} has no process() method"
            )

        self.filters[stage].append(
            (str(class_name), filter_object)
        )

        if self.logger is not None:
            self.logger.info(
                f"Loaded {stage} filter: {class_name}"
            )

    def apply(self, stage, data, context):
        if stage not in self.filters:
            raise ValueError(f"Unknown filter stage: {stage}")

        current = data

        for class_name, filter_object in self.filters[stage]:
            current = filter_object.process(
                current,
                context,
            )

            if current is None and stage != "result":
                raise RuntimeError(
                    f"{class_name} returned None in {stage} stage"
                )

        return current

    def describe(self):
        return {
            stage: [
                class_name
                for class_name, _ in self.filters[stage]
            ]
            for stage in self.VALID_STAGES
        }
