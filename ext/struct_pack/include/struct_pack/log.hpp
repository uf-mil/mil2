#pragma once

#include <array>
#include <chrono>
#include <format>
#include <iostream>
#include <source_location>
#include <span>
#include <string_view>

namespace print_hpp::log {

enum class LogLevel {
    TRACE,
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL,
};

enum class LogStyle {
    FG,
    BG,
};

namespace detail {
    constexpr auto to_string(LogLevel level) noexcept -> std::string_view {
        using enum LogLevel;
        switch (level) {
        case TRACE:
            return "TRACE";
        case DEBUG:
            return "DEBUG";
        case INFO:
            return "INFO ";
        case WARN:
            return "WARN ";
        case ERROR:
            return "ERROR";
        case FATAL:
            return "FATAL";
        default:
            return "UNKNOWN";
        }
    }

    constexpr auto to_color_fg(LogLevel level) noexcept -> std::string_view {
        using enum LogLevel;
        switch (level) {
        case TRACE:
            return "\033[36m";
        case DEBUG:
            return "\033[94m";
        case INFO:
            return "\033[32m";
        case WARN:
            return "\033[33m";
        case ERROR:
            return "\033[31m";
        case FATAL:
            return "\033[35m";
        default:
            return "";
        }
    }

    constexpr auto to_color_bg(LogLevel level) noexcept -> std::string_view {
        switch (level) {
            using enum LogLevel;
        case TRACE:
            return "\033[97;46m"; // cyan
        case DEBUG:
            return "\033[97;44m"; // blue
        case INFO:
            return "\033[97;42m"; // green
        case WARN:
            return "\033[90;43m"; // yellow
        case ERROR:
            return "\033[97;41m"; // red
        case FATAL:
            return "\033[97;45m"; // purple
        default:
            return "";
        }
    }

    constexpr auto to_color(LogLevel level, LogStyle style) noexcept
        -> std::string_view {
        using enum LogStyle;
        switch (style) {
        case FG:
            return to_color_fg(level);
        case BG:
            return to_color_bg(level);
        default:
            return "";
        }
    }

    template <LogLevel Level>
    auto function_name(std::source_location loc) noexcept -> std::string {
        if constexpr (Level <= LogLevel::DEBUG) {
            return std::format("|{}|", loc.function_name());
        } else {
            return {};
        }
    }

    constexpr auto source_color() noexcept -> std::string_view {
        return "\033[90m";
    }

    constexpr auto reset_color() noexcept -> std::string_view {
        return "\033[0m";
    }

    template <int N, char c>
    inline auto to_int(uint64_t num, std::span<char> p, int &size) {
        constexpr static std::array<char, 10> digits
            = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};

        for (int i = 0; i < N; i++) {
            p[--size] = digits.at(num % 10);
            num = num / 10;
        }

        if constexpr (N != 4) {
            p[--size] = c;
        }
    }

    inline auto get_timestamp(const std::chrono::system_clock::time_point &now)
        -> char * {
        static thread_local std::array<char, 33> buf{};
        static thread_local std::chrono::seconds last_second{};

        std::chrono::system_clock::duration duration = now.time_since_epoch();
        std::chrono::seconds                seconds
            = std::chrono::duration_cast<std::chrono::seconds>(duration);
        auto milliseconds
            = std::chrono::duration_cast<std::chrono::milliseconds>(duration
                                                                    - seconds)
                  .count();
        int size = 23;
        if (last_second == seconds) {
            to_int<3, '.'>(milliseconds, buf, size);
            return buf.data();
        }

        last_second = seconds;
        auto tt = std::chrono::system_clock::to_time_t(now);
        auto tm = localtime(&tt);

        to_int<3, '.'>(milliseconds, buf, size);
        to_int<2, ':'>(tm->tm_sec, buf, size);
        to_int<2, ':'>(tm->tm_min, buf, size);
        to_int<2, ' '>(tm->tm_hour, buf, size);

        to_int<2, '-'>(tm->tm_mday, buf, size);
        to_int<2, '-'>(tm->tm_mon + 1, buf, size);
        to_int<4, ' '>(tm->tm_year + 1900, buf, size);
        return buf.data();
    }

    class FmtWithSourceLocation {
    public:
        template <typename T>
            requires std::constructible_from<std::string_view, T>
        FmtWithSourceLocation(T                  &&fmt,
                              std::source_location location
                              = std::source_location::current())
            : fmt_{std::forward<T>(fmt)}
            , source_location_{location} {}

    public:
        auto fmt() {
            return fmt_;
        }

        auto source_location() {
            return source_location_;
        }

    private:
        std::string_view     fmt_;
        std::source_location source_location_;
    };

    class ConsoleLogger {
    public:
        template <typename... Args>
        auto trace(FmtWithSourceLocation fwsl, Args &&...args) {
            log<LogLevel::TRACE>(fwsl, std::forward<Args>(args)...);
        }

        template <typename... Args>
        auto debug(FmtWithSourceLocation fwsl, Args &&...args) {
            log<LogLevel::DEBUG>(fwsl, std::forward<Args>(args)...);
        }

        template <typename... Args>
        auto info(FmtWithSourceLocation fwsl, Args &&...args) {
            log<LogLevel::INFO>(fwsl, std::forward<Args>(args)...);
        }

        template <typename... Args>
        auto warn(FmtWithSourceLocation fwsl, Args &&...args) {
            log<LogLevel::WARN>(fwsl, std::forward<Args>(args)...);
        }

        template <typename... Args>
        auto error(FmtWithSourceLocation fwsl, Args &&...args) {
            log<LogLevel::ERROR>(fwsl, std::forward<Args>(args)...);
        }

        template <typename... Args>
        auto fatal(FmtWithSourceLocation fwsl, Args &&...args) {
            log<LogLevel::FATAL>(fwsl, std::forward<Args>(args)...);
        }

        template <typename Fmt, typename... Args>
            requires std::constructible_from<std::string_view, Fmt>
        auto printf(Fmt fmt, Args... args) {
            std::cout << std::vformat(fmt, std::make_format_args(args...));
        }

        template <typename Fmt, typename... Args>
            requires std::constructible_from<std::string_view, Fmt>
        auto printfln(Fmt fmt, Args... args) {
            std::cout << std::vformat(fmt, std::make_format_args(args...))
                      << '\n';
        }

        auto set_level(LogLevel level) {
            level_ = level;
        }

        auto set_style(LogStyle style) {
            style_ = style;
        }

    private:
        template <LogLevel Level, typename... Args>
        auto log(FmtWithSourceLocation fwsl, const Args &...args) {
            if (Level < level_) {
                return;
            }
            auto fmt = fwsl.fmt();
            auto source_location = fwsl.source_location();
            auto message = std::vformat(fmt, std::make_format_args(args...));
            auto now = std::chrono::system_clock::now();
            std::clog << std::format("{} |{}{:<5}{}| {}{}:{}{}{} {}\n",
                                     get_timestamp(now),
                                     to_color(Level, style_),
                                     to_string(Level),
                                     reset_color(),
                                     source_color(),
                                     source_location.file_name(),
                                     source_location.line(),
                                     function_name<Level>(source_location),
                                     reset_color(),
                                     message);
        }

    private:
        LogLevel level_{LogLevel::DEBUG};
        LogStyle style_{LogStyle::FG};
    };

} // namespace detail

inline auto console = detail::ConsoleLogger();
} // namespace print_hpp::log
