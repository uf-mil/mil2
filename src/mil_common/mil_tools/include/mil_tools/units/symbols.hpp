#pragma once

#include "au/unit_symbol.hh"
#include "au/units/meters.hh"

// These are defined in the global namespace so we can write
// expressions like 3.5 * m / s. Please be careful when
// including this file in other files.

constexpr auto m = au::symbol_for(au::meters);
