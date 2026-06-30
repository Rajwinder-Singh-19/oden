#pragma once
#include <cmath>
#include <concepts>
#include <functional>
#include <iostream>
#include <ranges>
#include <vector>

namespace oden
{
struct ControlParameters
{
    double startTime;
    double endTime;
    double stepSizeMax;
    double stepSizeMin;
    double atol;
    double rtol;
};

struct Solution
{
    double time;
    std::vector<double> variables;
};

template <typename... Vector>
    requires((std::same_as<Vector, std::vector<double>>) && ...)
class System
{
  public:
    using State = std::vector<double>;
    using StateView = std::span<const double>;
    using Index = size_t;
    using IndexList = std::vector<Index>;
    using StateFunction = std::function<std::vector<double>(double, const System &)>;
    using SolutionVec = std::vector<Solution>;

  private:
    State mGlobalState;
    IndexList mVarORDERS;
    IndexList mVarOFFSETS;
    StateFunction mDerivFunc;

    State mGetVarStateVec(size_t varIndex)
    {
        auto stateSpan = this->operator[](varIndex);      // Const double span which contains the variable state
        return State(stateSpan.begin(), stateSpan.end()); //  Converts span into a vector and returns
    }

    State mGetDerivatives(double time, const System &currSystem)
    {
        return mDerivFunc(time, currSystem);
    }

    void mSetGlobalState(const State &state)
    {
        mGlobalState = state;
    }

    State mGetGlobalSol(const State &state) const
    {
        State globalSol;
        globalSol.resize(mVarORDERS.size());
        for (const auto [varIdx, varOffset] : std::views::enumerate(mVarOFFSETS))
        {
            globalSol[varIdx] = state[varOffset];
        }

        return globalSol;
    }
    std::array<double, 2> mAcceptStepSize(const State &currState, State &fullStepSizeState,
                                          const State &halfStepSizeState, double atol, double rtol)
    {
        size_t numVars = mVarORDERS.size();

        State currSol = mGetGlobalSol(currState);
        State fullStepSol = mGetGlobalSol(fullStepSizeState);
        State halfStepSol = mGetGlobalSol(halfStepSizeState);
        double globalErrorScalar = 0;
        for (const auto varIdx : std::views::iota(size_t{0}, numVars))
        {
            double varError = std::fabs(fullStepSol[varIdx] - halfStepSol[varIdx]);
            double varScale = atol + rtol * std::max(currSol[varIdx], halfStepSol[varIdx]);
            double varNorm = varError / varScale;
            globalErrorScalar += std::pow(varNorm, 2.0);
        }

        globalErrorScalar /= numVars;

        globalErrorScalar = std::sqrt(globalErrorScalar);

        if (globalErrorScalar <= 1)
        {
            return std::array<double, 2>{1.0, globalErrorScalar};
        }

        return std::array<double, 2>{0.0, globalErrorScalar};
    }

    State mEulerStepLocal(size_t varIdx, double time, double stepSize, const State &globalState,
                          const State &varDerivState)
    {
        double varDeriv = varDerivState[varIdx];
        size_t varOrd = mVarORDERS[varIdx];

        State varState = mGetVarStateVec(varIdx);

        for (size_t i = 0; i < varOrd - 1; i++)
        {
            varState[i] += stepSize * varState[i + 1];
        }
        varState[varOrd - 1] += stepSize * varDeriv;

        return varState;
    }

    State mEulerStepGlobal(double time, double stepSize, const State &globalState)
    {
        const System &currSystem = *this;
        State varDerivState = mGetDerivatives(time, currSystem);
        State newGlobalState;

        newGlobalState.resize(mGlobalState.size());

        for (const auto [varIdx, varOffset] : std::views::enumerate(mVarOFFSETS))
        {
            State varState = mEulerStepLocal(varIdx, time, stepSize, globalState, varDerivState);
            std::copy_n(varState.begin(), varState.size(), newGlobalState.begin() + varOffset);
        }

        return newGlobalState;
    }

  public:
    System(const Vector &...initialStates)
    {
        size_t varOffset = 0;
        auto insertOrders = [&](const auto &state) {
            mVarOFFSETS.push_back(varOffset);
            size_t varOrd = state.size();
            mVarORDERS.push_back(varOrd);
            varOffset += varOrd;
        };

        (insertOrders(initialStates), ...);

        mGlobalState.resize(varOffset);

        varOffset = 0;

        auto insertStates = [&](const auto &state) {
            size_t varOrd = state.size();
            std::copy_n(state.begin(), varOrd, mGlobalState.begin() + varOffset);
            varOffset += varOrd;
        };

        (insertStates(initialStates), ...);
    }
    StateView operator[](size_t varIndex) const
    {
        size_t varOffset = mVarOFFSETS[varIndex];
        size_t varOrd = mVarORDERS[varIndex];
        return StateView(mGlobalState.data() + varOffset, varOrd);
    }

    void bindStateFunction(const StateFunction &stateFunction)
    {
        mDerivFunc = stateFunction;
    }

    SolutionVec EulerSolver(const ControlParameters &controlParameters, bool adaptiveSwitch)
    {
        double time = controlParameters.startTime;
        double endTime = controlParameters.endTime;
        double stepSize = controlParameters.stepSizeMax;
        double atol = controlParameters.atol;
        double rtol = controlParameters.rtol;
        double stepSizeMin = controlParameters.stepSizeMin;
        SolutionVec solution;
        solution.reserve(static_cast<size_t>((endTime - time) / stepSize));
        solution.emplace_back(Solution{time, mGetGlobalSol(mGlobalState)});
        State fullStepState;
        State halfStepState1;
        State halfStepState2;

        State solFullStep;
        State solHalfStep;

        while (std::fabs(time - endTime) > 1e-6 && time < endTime)
        {
            double halfStepSize = 0.5 * stepSize;
            fullStepState = mEulerStepGlobal(time, stepSize, mGlobalState);
            halfStepState1 = mEulerStepGlobal(time, halfStepSize, mGlobalState);
            halfStepState2 = mEulerStepGlobal(time + halfStepSize, halfStepSize, halfStepState1);

            solFullStep = mGetGlobalSol(fullStepState);
            solHalfStep = mGetGlobalSol(halfStepState2);

            std::array<double, 2> stepDecisionArray =
                mAcceptStepSize(mGlobalState, fullStepState, halfStepState2, atol, rtol);
            
                bool acceptStepSize = true;
            if (adaptiveSwitch && (std::fabs(stepSize - stepSizeMin) > atol))
            {
                acceptStepSize = static_cast<bool>(stepDecisionArray[0]);
                double stepSizeNew;
                if (!acceptStepSize)
                {
                    stepSizeNew = 0.9 * stepSize * std::pow(1.0 / stepDecisionArray[1], 0.5);
                    stepSize = std::max(stepSizeNew, stepSizeMin);
                    continue;
                } else
                {
                
                }
                
            }

            time += stepSize;
            solution.emplace_back(Solution{time, solHalfStep});
            mSetGlobalState(halfStepState2);
        }

        for (auto &&sol : solution)
        {
            std::cout << sol.time << ",";

            for (auto &&val : sol.variables)
            {
                std::cout << val << ",";
            }

            std::cout << std::endl;
        }

        return solution;
    }

    ~System() = default;
};
} // namespace oden