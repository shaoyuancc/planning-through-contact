from typing import List, Tuple

import numpy as np
import numpy.typing as npt
import pydrake.symbolic as sym
from pydrake.math import eq, ge
from pydrake.solvers import Binding, MathematicalProgram, Solve

from tools.types import NpMonomialArray, NpVariableArray


# TODO there is definitely a much more efficient way of doing this
def _linear_binding_to_formula(binding: Binding) -> sym.Formula:
    """
    Takes in a binding and returns a polynomial p that should satisfy p(x) = 0
    """
    # NOTE: I cannot use binding.evaluator().Eval(binding.variables())
    # here, because it ignores the constant term for linear constraints! Is this a bug?
    A = binding.evaluator().GetDenseA()
    b_upper = (
        binding.evaluator().upper_bound()
    )  # TODO for equalities, these seem to be the same!
    x = binding.variables()
    formula = A.dot(x) - b_upper
    return formula


def _linear_bindings_to_homogenuous_form(
    linear_bindings: List[Binding], vars: NpVariableArray
) -> npt.NDArray[np.float64]:
    linear_formulas = np.array([_linear_binding_to_formula(b) for b in linear_bindings])
    A, b = sym.DecomposeAffineExpressions(linear_formulas, vars)
    A_homogenous = np.hstack((b.reshape(-1, 1), A))
    return A_homogenous


def _generic_binding_to_polynomial(binding: Binding) -> sym.Polynomial:
    poly = sym.Polynomial(binding.evaluator().Eval(binding.variables())[0])
    return poly


def _get_monomial_coeffs(
    poly: sym.Polynomial, basis: NpMonomialArray
) -> npt.NDArray[np.float64]:
    coeff_map = poly.monomial_to_coefficient_map()
    coeffs = np.array([coeff_map.get(m, sym.Expression(0)).Evaluate() for m in basis])
    return coeffs


def _construct_symmetric_matrix_from_triang(
    triang_matrix: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    return triang_matrix + triang_matrix.T


def _quadratic_polynomial_to_homoenuous_form(
    poly: sym.Polynomial, basis: NpMonomialArray, num_vars: int
) -> npt.NDArray[np.float64]:
    coeffs = _get_monomial_coeffs(poly, basis)
    upper_triangular = np.zeros((num_vars, num_vars))
    upper_triangular[np.triu_indices(num_vars)] = coeffs
    Q = _construct_symmetric_matrix_from_triang(upper_triangular)
    return Q * 0.5


def create_sdp_relaxation(prog: MathematicalProgram) -> MathematicalProgram:
    DEGREE_QUADRATIC = 2  # We are only relaxing (non-convex) quadratic programs

    decision_vars = prog.decision_variables()
    num_vars = (
        len(decision_vars) + 1
    )  # 1 will also be a decision variable in the relaxation

    basis = np.flip(sym.MonomialBasis(decision_vars, DEGREE_QUADRATIC))
    relaxed_prog = MathematicalProgram()
    X = relaxed_prog.NewSymmetricContinuousVariables(num_vars, "X")
    relaxed_prog.AddPositiveSemidefiniteConstraint(X)

    relaxed_prog.AddConstraint(X[0, 0] == 1)  # First variable is 1

    has_linear_eq_constraints = len(prog.linear_equality_constraints()) > 0
    if has_linear_eq_constraints:
        A_eq = _linear_bindings_to_homogenuous_form(
            prog.linear_equality_constraints(), decision_vars
        )
        relaxed_prog.AddLinearConstraint(eq(A_eq.dot(X).dot(A_eq.T), 0))

    has_linear_ineq_constraints = len(prog.linear_constraints()) > 0
    if has_linear_ineq_constraints:
        A_ineq = _linear_bindings_to_homogenuous_form(
            prog.linear_constraints(), decision_vars
        )
        relaxed_prog.AddLinearConstraint(
            ge(A_ineq.dot(X).dot(A_ineq.T), 0)
        )  # TODO handle upper bounds too!

    has_generic_constaints = len(prog.generic_constraints()) > 0
    if has_generic_constaints:
        # TODO differentiate between eq and ineq
        generic_constraints_as_polynomials = [
            _generic_binding_to_polynomial(b) for b in prog.generic_constraints()
        ]

        max_degree = max([p.TotalDegree() for p in generic_constraints_as_polynomials])
        min_degree = min([p.TotalDegree() for p in generic_constraints_as_polynomials])
        if max_degree > DEGREE_QUADRATIC or min_degree < DEGREE_QUADRATIC:
            raise ValueError(
                "Can only create SDP relaxation for (possibly non-convex) Quadratically Constrainted Quadratic Programs (QCQP)"
            )  # TODO for now we don't allow lower degree or higher degree

        Q_eqs = [
            _quadratic_polynomial_to_homoenuous_form(p, basis, num_vars)
            for p in generic_constraints_as_polynomials
        ]
        for Q in Q_eqs:
            constraints = eq(np.trace(X.dot(Q)), 0).flatten()
            for c in constraints:  # Drake requires us to add one constraint at the time
                relaxed_prog.AddLinearConstraint(c)

    breakpoint()

    return relaxed_prog


def add_constraint(self, formula: sym.Formula, bp: bool = False) -> None:
    kind = formula.get_kind()
    lhs, rhs = formula.Unapply()[1]  # type: ignore
    poly = sym.Polynomial(lhs - rhs)

    if poly.TotalDegree() > self.degree:
        raise ValueError(
            f"Constraint degree is {poly.TotalDegree()},"
            "program degree is {self.degree}"
        )

    Q = self._construct_quadratic_constraint(poly, self.mon_basis, self.n)
    constraint_lhs = np.trace(self.X @ Q)
    if bp:
        breakpoint()
    if kind == sym.FormulaKind.Eq:
        self.prog.AddConstraint(constraint_lhs == 0)
    elif kind == sym.FormulaKind.Geq:
        self.prog.AddConstraint(constraint_lhs >= 0)
    elif kind == sym.FormulaKind.Leq:
        self.prog.AddConstraint(constraint_lhs <= 0)
    else:
        raise NotImplementedError(f"Support for formula type {kind} not implemented")


class SdpRelaxation:
    def __init__(self, vars: npt.NDArray[sym.Variable]):
        self.n = vars.shape[0] + 1  # 1 is also a monomial
        self.order = 1  # For now, we just do the first order of the hierarchy

        # [1, x, x ** 2, ... ]
        self.mon_basis = np.flip(sym.MonomialBasis(vars, self.degree))

        self.prog = MathematicalProgram()
        self.X = self.prog.NewSymmetricContinuousVariables(self.n, "X")
        self.prog.AddConstraint(
            self.X[0, 0] == 1
        )  # First variable is not really a variable
        self.prog.AddPositiveSemidefiniteConstraint(self.X)

    @property
    def degree(self) -> int:
        return self.order + 1

    def add_constraint(self, formula: sym.Formula, bp: bool = False) -> None:
        kind = formula.get_kind()
        lhs, rhs = formula.Unapply()[1]  # type: ignore
        poly = sym.Polynomial(lhs - rhs)

        if poly.TotalDegree() > self.degree:
            raise ValueError(
                f"Constraint degree is {poly.TotalDegree()},"
                "program degree is {self.degree}"
            )

        Q = self._construct_quadratic_constraint(poly, self.mon_basis, self.n)
        constraint_lhs = np.trace(self.X @ Q)
        if bp:
            breakpoint()
        if kind == sym.FormulaKind.Eq:
            self.prog.AddConstraint(constraint_lhs == 0)
        elif kind == sym.FormulaKind.Geq:
            self.prog.AddConstraint(constraint_lhs >= 0)
        elif kind == sym.FormulaKind.Leq:
            self.prog.AddConstraint(constraint_lhs <= 0)
        else:
            raise NotImplementedError(
                f"Support for formula type {kind} not implemented"
            )

    def get_solution(self) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        result = Solve(self.prog)
        assert result.is_success()
        X_result = result.GetSolution(self.X)
        svd_solution = self._get_sol_from_svd(X_result)
        variable_values = svd_solution[1:]  # first value is 1
        return variable_values, X_result

    def _get_sol_from_svd(self, X: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        eigenvals, eigenvecs = np.linalg.eig(X)
        idx_highest_eigval = np.argmax(eigenvals)
        solution_nonnormalized = eigenvecs[:, idx_highest_eigval]
        solution = solution_nonnormalized / solution_nonnormalized[0]
        return solution

    def _get_monomial_coeffs(
        self, poly: sym.Polynomial, basis: npt.NDArray[sym.Monomial]
    ):
        coeff_map = poly.monomial_to_coefficient_map()
        breakpoint()
        coeffs = np.array(
            [coeff_map.get(m, sym.Expression(0)).Evaluate() for m in basis]
        )
        return coeffs

    def _construct_symmetric_matrix_from_triang(
        self,
        triang_matrix: npt.NDArray[np.float64],
    ) -> npt.NDArray[np.float64]:
        return triang_matrix + triang_matrix.T

    def _construct_quadratic_constraint(
        self, poly: sym.Polynomial, basis: npt.NDArray[sym.Monomial], n: int
    ) -> npt.NDArray[np.float64]:
        coeffs = self._get_monomial_coeffs(poly, basis)
        upper_triangular = np.zeros((n, n))
        upper_triangular[np.triu_indices(n)] = coeffs
        Q = self._construct_symmetric_matrix_from_triang(upper_triangular)
        return Q * 0.5
