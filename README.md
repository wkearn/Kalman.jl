# Kalman

[![Build Status](https://travis-ci.org/wkearn/Kalman.jl.svg?branch=master)](https://travis-ci.org/wkearn/Kalman.jl)
[![Coverage Status](https://coveralls.io/repos/wkearn/Kalman.jl/badge.png?branch=master)](https://coveralls.io/r/wkearn/Kalman.jl?branch=master)

A generic interface for Kalman filters in Julia.

Note that the [TimeModels.jl](https://github.com/JuliaStats/TimeModels.jl) also has an implementation of Kalman filters for time series analysis. This implementation (Kalman.jl) focuses on applications of Kalman filters to online data assimilation problems, and intends to develop a single API for both linear and nonlinear Kalman filters. Kalman.jl is an unregistered package still in active development, so bugs may be common and changes rapid. For a more stable and tested API, check out TimeModels.jl.
