import matplotlib.pyplot as plt
import cdd

from dataclasses import dataclass
import numpy as np
import numpy.typing as npt
from typing import List, Literal

import math
from pydrake.math import le, ge, eq
import pydrake.symbolic as sym
import pydrake.geometry.optimization as opt

from pydrake.geometry.optimization import GraphOfConvexSets
from pydrake.solvers import MathematicalProgram, Solve, MathematicalProgramResult

from geometry.polyhedron import Polyhedron
from geometry.bezier import BezierCurve, BezierVariable
from geometry.contact import ContactMode
from planning.gcs import GcsPlanner, GcsContactPlanner


def create_test_polyhedrons() -> List[Polyhedron]:
    vertices = [
        np.array([[0, 0], [0, 2], [1.5, 3], [2, 0]]),
        np.array([[1, 2], [1, 4.5], [3, 4.5], [4, 1]]),
        np.array([[3, 2], [3, 3], [5, 3], [5, 2]]),
        np.array([[3, 4], [3, 5], [6, 5], [6, 4]]),
        np.array([[5, 4], [7, 6], [8, 2.5], [4.5, 2.5]]),
        # For some reason, this violates continuity
        # np.array([[4, 4], [7, 6], [8, 2.5], [4.5, 2.5]]),
    ]
    polyhedrons = [Polyhedron.from_vertices(v) for v in vertices]

    return polyhedrons


def test_bezier_curve() -> None:
    order = 2
    dim = 2

    poly = create_test_polyhedron_1()
    vertices = poly.get_vertices()
    plt.fill(vertices[:, 0], vertices[:, 1], alpha=0.3)

    x0 = np.array([0, 0.5]).reshape((-1, 1))
    xf = np.array([4, 3]).reshape((-1, 1))

    bezier_curve = BezierCurveMathProgram(order, dim)
    bezier_curve.constrain_to_polyhedron(poly)
    bezier_curve.constrain_start_pos(x0)
    bezier_curve.constrain_end_pos(xf)
    bezier_curve.calc_ctrl_points()
    path = np.concatenate(
        [bezier_curve.eval(s) for s in np.arange(0.0, 1.01, 0.01)], axis=1
    ).T

    plt.plot(path[:, 0], path[:, 1])
    plt.scatter(x0[0], x0[1])
    plt.scatter(xf[0], xf[1])

    plt.show()


def plot_polyhedrons(polys: List[Polyhedron]) -> None:
    for poly in polys:
        vertices = poly.get_vertices()
        plt.fill(vertices[:, 0], vertices[:, 1], alpha=0.3)
    plt.show()


def test_gcs() -> None:
    order = 2
    dim = 2

    polys = create_test_polyhedrons()

    path = GcsPlanner(order, polys)

    x0 = np.array([0.5, 0.5]).reshape((-1, 1))
    xf = np.array([7.0, 5.5]).reshape((-1, 1))

    breakpoint()
    v0 = path.add_point_vertex(x0, "source", "out")
    vf = path.add_point_vertex(xf, "target", "in")
    ctrl_points = path.calculate_path(v0, vf)
    curves = [
        BezierCurve.create_from_ctrl_points(dim, points) for points in ctrl_points
    ]

    # Plotting
    for poly in polys:
        vertices = poly.get_vertices()
        plt.fill(vertices[:, 0], vertices[:, 1], alpha=0.3)

    for curve in curves:
        plt.scatter(curve.ctrl_points[0, :], curve.ctrl_points[1, :])

        curve_values = np.concatenate(
            [curve.eval(s) for s in np.arange(0.0, 1.01, 0.01)], axis=1
        ).T

        plt.plot(curve_values[:, 0], curve_values[:, 1])

    plt.show()

    return


def test_planning_through_contact():
    lam_n = BezierVariable(dim=1, order=2, name="lambda_n")
    lam_f = BezierVariable(dim=1, order=2, name="lambda_f")
    x_a = BezierVariable(dim=1, order=2, name="x_a")
    x_u = BezierVariable(dim=1, order=2, name="x_u")

    friction_coeff = 0.5
    contact_jacobian = np.array([[-1, 1]])
    normal_jacobian = contact_jacobian
    tangential_jacobian = -contact_jacobian

    pos_vars = np.array([x_a, x_u])
    normal_force_vars = np.array([lam_n])
    friction_force_vars = np.array([lam_f])

    l = 0.5

    no_contact_pos_constraint = x_a + l <= x_u
    no_contact = ContactMode(
        pos_vars,
        no_contact_pos_constraint,
        normal_force_vars,
        friction_force_vars,
        "no_contact",
        friction_coeff,
        normal_jacobian,
        tangential_jacobian,
    )
    contact_pos_constraint = x_a + l == x_u
    rolling_contact = ContactMode(
        pos_vars,
        contact_pos_constraint,
        normal_force_vars,
        friction_force_vars,
        "rolling_contact",
        friction_coeff,
        normal_jacobian,
        tangential_jacobian,
    )
    sliding_contact = ContactMode(
        pos_vars,
        contact_pos_constraint,
        normal_force_vars,
        friction_force_vars,
        "sliding_contact",
        friction_coeff,
        normal_jacobian,
        tangential_jacobian,
    )

    contact_modes = [no_contact, rolling_contact, sliding_contact]
    position_polyhedrons = [mode.convex_set_position for mode in contact_modes]

    # Make sure position sets are overlapping
    for p1, p2 in zip(position_polyhedrons[:-1], position_polyhedrons[1:]):
        assert p1.IntersectsWith(p2)


    planner = GcsContactPlanner([no_contact, rolling_contact, sliding_contact])

    # TODO: Now tie this to GCS
    breakpoint()


def main():
    # test_bezier_curve()
    # test_gcs()
    test_planning_through_contact()

    return 0


if __name__ == "__main__":
    main()
