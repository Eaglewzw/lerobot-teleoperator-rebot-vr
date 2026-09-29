"""Small strictly convex box QP, with a bounded amount of solver work."""

import itertools

import numpy as np


_STATES = np.array(list(itertools.product((-1, 0, 1), repeat=3)))
_FREE = _STATES == 0
_IDENTITY = np.eye(3)


def solve_box_qp3(hessian, gradient, lower, upper):
    """Minimize .5*x.T*H*x + g.T*x on a three-dimensional box.

    H must be positive definite. Each axis is free, at its lower bound,
    or at its upper bound: enumerate all 27 faces in one batched solve.
    The minimum feasible face stationary point is the global minimum.
    This also includes every corner and boxes with fixed coordinates.
    """
    matrices = np.where(_FREE[:, :, None], hessian, _IDENTITY)
    rhs = np.where(_FREE, -gradient, np.where(_STATES < 0, lower, upper))
    candidates = np.linalg.solve(matrices, rhs[..., None])[..., 0]
    feasible = np.all(
        (candidates >= lower - 1e-10) & (candidates <= upper + 1e-10), axis=1
    )
    candidates = np.clip(candidates, lower, upper)
    costs = .5 * np.einsum("bi,ij,bj->b", candidates, hessian, candidates)
    costs += candidates @ gradient
    costs[~feasible] = np.inf
    if not np.any(np.isfinite(costs)):
        raise ValueError("no finite feasible box QP candidate")
    return candidates[np.argmin(costs)].copy()
