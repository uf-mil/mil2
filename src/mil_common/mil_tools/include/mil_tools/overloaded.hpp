#pragma once

namespace mil::overloaded
{
template <class... Ts>
struct make : Ts...
{
    using Ts::operator()...;
};
template <class... Ts>
make(Ts...) -> make<Ts...>;
}  // namespace mil::overloaded
