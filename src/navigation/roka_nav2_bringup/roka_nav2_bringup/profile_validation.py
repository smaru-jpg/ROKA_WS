"""Fail-closed validation shared by PK-3A autonomy launch and follower."""


def is_autonomy_approved(document):
    """Only an explicit YAML boolean true authorizes autonomous motion."""
    return isinstance(document, dict) and document.get("autonomy_approved") is True


def is_test_only_profile(document):
    """Return true only for an explicitly scoped controlled-area test path."""
    return (
        isinstance(document, dict)
        and document.get("test_only") is True
        and document.get("approval_scope") == "controlled_short_autonomy_test"
    )


def is_profile_allowed(document, allow_test_profile=False):
    """Require normal approval and an extra opt-in for test-only profiles."""
    if not is_autonomy_approved(document):
        return False
    return not is_test_only_profile(document) or bool(allow_test_profile)
