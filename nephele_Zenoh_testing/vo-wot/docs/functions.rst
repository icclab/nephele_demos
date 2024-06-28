VO Virtual functions
====================

This page describes the available VO Functions that can be activated.

Available Generic Functions
^^^^^^^^^^^^^^^^^^^^^^^^^^^

================= =====================================================
Function Name     Description
================= =====================================================
**forecasting**   Forecasts the next value of a property using
                  an ARIMA model.
**mean_value**    Calculates the mean value of a property for a
                  specific time window.
**vo_status**     Attempts to access the catalogue port of the
                  VO to check if it's running correctly.
**device_status** Attempts to access the device to check if it's
                  running correctly.
================= =====================================================

By listing the desired Generic Functions inside the Virtual Object Descriptor,
the function are then injected in the developer's python script.

Custom Functions
^^^^^^^^^^^^^^^^

Custom functions can be directly defined inside of the python script and called from there.
