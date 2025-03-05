#pragma once

namespace mil_tools::overloaded
{
template <class... Ts>
struct make : Ts...
{
    using Ts::operator()...;
};
template <class... Ts>
make(Ts...) -> make<Ts...>;
}  // namespace mil_tools::overloaded
