function [FVAL] = UAV_model_implicit(XDOT, X, U)

FVAL = UAV_model(X,U) - XDOT;