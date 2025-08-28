#pragma once

#include <numeric>

#include "au/prefix.hh"
#include "au/unit_symbol.hh"
#include "au/units/degrees.hh"
#include "au/units/feet.hh"
#include "au/units/inches.hh"
#include "au/units/meters.hh"
#include "au/units/radians.hh"

// These are defined in the global namespace so we can write
// expressions like 3.5 * m / s. Please be careful when
// including this file in other files.

// Distances
constexpr auto in = au::symbol_for(au::inches);
constexpr auto m = au::symbol_for(au::meters);
constexpr auto km = au::symbol_for(au::kilo(au::meters));
constexpr auto ft = au::symbol_for(au::feet);

// Angles
constexpr auto deg = au::symbol_for(au::degrees);
constexpr auto rad = au::symbol_for(au::radians);
