from ._core import mr_planning as _mr_planning

# List of expected symbols to always export if present
_expected_exports = [
    "FloatVector7",
    "MRProblem",
    "MRPlanningResult",
    "MRSettings",
    "MRPlannerBase",
    "DummyMRPlanner",
    "MRPlannerFactory",
    "create_mr_problem",
    "solve_mr_problem",
    "create_dummy_planner",
]

__all__ = list({
    *(getattr(_mr_planning, "__all__", [])),
    *_expected_exports
})

globals().update({k: getattr(_mr_planning, k) for k in __all__ if hasattr(_mr_planning, k)}) 