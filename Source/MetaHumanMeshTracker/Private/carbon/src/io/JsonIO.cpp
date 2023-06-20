// Copyright Epic Games, Inc. All Rights Reserved.

#include <carbon/io/JsonIO.h>

#include <cstring>
#include <sstream>
#include <algorithm>

namespace epic {
namespace carbon {
// ! Check if a character is a whitespace as allowed in JSON
inline bool IsWhitespace(const char c) {
    return (c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\0');
}

// ! Check if a character is a digit
inline bool IsDigit(const char c) {
    return (c >= '0' && c <= '9');
}

// ! Skip any whitespace character.
inline void SkipWhitespace(const char* buf, size_t& pos, size_t end) {
    while (pos < end && IsWhitespace(buf[pos])) {
        pos++;
    }
}

// ! Parse string and check if it the expected string.
template<size_t N>
inline void ParseAndExpectString(const char* buf, size_t& pos, size_t end, const char (& expectedString)[N]) {
    constexpr size_t N1 = N - 1;
    if (pos + N1 <= end) {
        if (strncmp(buf + pos, expectedString, N1)) {
            CARBON_CRITICAL("json parser: expecting '{}', but got '{}'", std::string(expectedString, N1),
                            std::string(buf + pos, N1));
        }
        pos += N1;
    } else {
        CARBON_CRITICAL("json parser: unexpected end of stream while parsing for '{}'", std::string(expectedString, N1));
    }
}

// ! Check if the current character represents a valid end of a number i.e. whitespace, comma, or closing square or
// curling
// brackets
bool IsEndOfNumber(const char* buf, size_t pos, size_t end) {
    if (pos < end) {
        const char c = buf[pos];
        return IsWhitespace(c) || c == ',' || c == ']' || c == '}';
    }
    return true;
}

// ! Read a JsonElement of Type JsonType::Number. Expects the context to contain this type.
JsonElement ReadNumberJsonElement(const char* buf, size_t& pos, size_t end) {
    if (pos < end) {
        const size_t startPos = pos;
        // check for minus sign
        const bool isNegative = (buf[pos] == '-');
        if (isNegative) {
            pos++;
        }
        // check int part
        if (pos >= end) {
            CARBON_CRITICAL("json parser: unexpected end of stream while parsing for number");
        }
        if (!IsDigit(buf[pos])) {
            CARBON_CRITICAL("json parser: syntax error while parsing number; expected digit but read {}", buf[pos]);
        }
        pos++;
        while (pos < end && IsDigit(buf[pos])) {
            pos++;
        }

        if (IsEndOfNumber(buf, pos, end)) {
            // read integer
            return JsonElement(std::stoi(std::string(buf + startPos, pos - startPos)));
        }

        CARBON_ASSERT(pos < end, "we should not be at the end of the stream");

        // check for fraction
        bool hasFraction = (buf[pos] == '.');
        if (hasFraction) {
            pos++;
            if (pos < end) {
                if (!IsDigit(buf[pos])) {
                    CARBON_CRITICAL(
                        "json parser: syntax error while parsing number; expected digit after '.' but read {}",
                        buf[pos]);
                }
                while (pos < end && IsDigit(buf[pos])) {
                    pos++;
                }
            } else {
                CARBON_CRITICAL("json parser: unexpected end of stream while parsing for fractional part of number");
            }
        }

        if (IsEndOfNumber(buf, pos, end)) {
            // read double
            return JsonElement(std::stod(std::string(buf + startPos, pos - startPos)));
        }

        CARBON_ASSERT(pos < end, "we should not be at the end of the stream");

        // check for exp
        const char c = buf[pos];
        if ((c == 'e') || (c == 'E')) {
            pos++;
            if (pos < end) {
                if ((buf[pos] == '+') || (buf[pos] == '-')) {
                    pos++;
                }
            }
            if (pos < end) {
                if (!IsDigit(buf[pos])) {
                    CARBON_CRITICAL(
                        "json parser: syntax error while parsing number; expected digit after 'e' but read {}",
                        buf[pos]);
                }
                while (pos < end && IsDigit(buf[pos])) {
                    pos++;
                }
                // read double
                return JsonElement(std::stod(std::string(buf + startPos, pos - startPos)));
            } else {
                CARBON_CRITICAL("json parser: unexpected end of stream while parsing for exponential part of number");
            }
        } else {
            CARBON_CRITICAL("json parser: unexpected symbol '{}' while parsing for exponential part of number {}", c,
                            std::string(buf + startPos, pos + 1 - startPos));
        }
    } else {
        CARBON_CRITICAL("json parser: unexpected end of stream while parsing for number");
    }
}

// ! Read a JsonElement of Type JsonType::String. Expects the context to contain this type.
JsonElement ReadStringJsonElement(const char* buf, size_t& pos, size_t end) {
    CARBON_ASSERT(pos < end, "json parser: ReadStringJsonElement expects at least one character to read");
    CARBON_ASSERT(buf[pos] == '\"', "json parser: ReadStringJsonElement expects first character to be '\"'");
    pos++;
    size_t startPos = pos;
    // parse for closing quote
    while (true) {
        bool ignoreNext = false;
        if (pos < end) {
            if ((buf[pos] == '\"') && !ignoreNext) {
                pos++;
                return JsonElement(std::string(buf + startPos, pos - 1 - startPos));
            }
            if ((buf[pos] == '\\') && !ignoreNext) {
                ignoreNext = true;
            }
            pos++;
        } else {
            CARBON_CRITICAL("json parser: unexpected of end of stream while parsing for closing quote");
        }
    }
}

JsonElement ReadJsonElement(const char* buf, size_t& pos, size_t end);

// ! Read a JsonElement of Type JsonType::Array. Expects the context to contain this type.
JsonElement ReadArrayJsonElement(const char* buf, size_t& pos, size_t end) {
    CARBON_ASSERT(pos < end, "json parser: ReadArrayJsonElement expects at least one character to read");
    CARBON_ASSERT(buf[pos] == '[', "json parser: ReadArrayJsonElement expects first character to be '['");
    pos++;
    std::vector<JsonElement> arrayElements;
    while (true) {
        SkipWhitespace(buf, pos, end);
        if (pos < end) {
            if (buf[pos] == ']') {
                pos++;
                // closing quotes
                return JsonElement(std::move(arrayElements));
            } else {
                arrayElements.emplace_back(ReadJsonElement(buf, pos, end));
                SkipWhitespace(buf, pos, end);
                if (pos < end) {
                    if (buf[pos] == ',') {
                        pos++;
                        SkipWhitespace(buf, pos, end);
                    }
                }
            }
        } else {
            CARBON_CRITICAL("json parser: unexpected end of stream while parsing array");
        }
    }
}

// ! Read a JsonElement of Type JsonType::Object. Expects the context to contain this type.
JsonElement ReadObjectJsonElement(const char* buf, size_t& pos, size_t end) {
    CARBON_ASSERT(pos < end, "json parser: ReadObjectJsonElement expects at least one character to read");
    CARBON_ASSERT(buf[pos] == '{', "json parser: ReadObjectJsonElement expects first character to be '{'");
    pos++;
    std::map<std::string, JsonElement> objectElements;
    while (true) {
        SkipWhitespace(buf, pos, end);
        if (pos < end) {
            if (buf[pos] == '}') {
                pos++;
                // closing quotes
                return JsonElement(std::move(objectElements));
            } else {
                JsonElement keyElement = ReadJsonElement(buf, pos, end);
                if (!keyElement.IsString()) {
                    CARBON_CRITICAL("json parser: object does not contain a string key");
                }
                const std::string key = keyElement.String();
                if (objectElements.find(key) != objectElements.end()) {
                    CARBON_CRITICAL("json parser: object already contains key \"{}\"", key);
                }
                SkipWhitespace(buf, pos, end);
                if (pos >= end) {
                    CARBON_CRITICAL("json parser: unexpected end of stream while parsing for ':'");
                }
                if (buf[pos] != ':') {
                    CARBON_CRITICAL("json parser: unexpected symbol '{}' while parsing for ':'", buf[pos]);
                }
                pos++;
                JsonElement element = ReadJsonElement(buf, pos, end);
                objectElements.emplace(key, std::move(element));
                SkipWhitespace(buf, pos, end);
                if (pos < end) {
                    if (buf[pos] == ',') {
                        pos++;
                        SkipWhitespace(buf, pos, end);
                    }
                }
            }
        } else {
            CARBON_CRITICAL("json parser: unexpected end of stream while parsing array");
        }
    }
}

JsonElement ReadJsonElement(const char* buf, size_t& pos, size_t end) {
    SkipWhitespace(buf, pos, end);
    if (pos < end) {
        const char c = buf[pos];
        if (c == '{') {
            // object
            return ReadObjectJsonElement(buf, pos, end);
        } else if (c == '[') {
            // array
            return ReadArrayJsonElement(buf, pos, end);
        } else if (c == '\"') {
            // expect string
            return ReadStringJsonElement(buf, pos, end);
        } else if (c == 'f') {
            // expect false
            ParseAndExpectString(buf, pos, end, "false");
            return JsonElement(JsonElement::JsonType::False);
        } else if (c == 't') {
            // expect true
            ParseAndExpectString(buf, pos, end, "true");
            return JsonElement(JsonElement::JsonType::True);
        } else if (c == 'n') {
            // expect null
            ParseAndExpectString(buf, pos, end, "null");
            return JsonElement(JsonElement::JsonType::Null);
        } else if (c == '-') {
            return ReadNumberJsonElement(buf, pos, end);
        } else if (IsDigit(c)) {
            return ReadNumberJsonElement(buf, pos, end);
        } else {
            CARBON_CRITICAL("json parser: unexpected character '{}' as start token", c);
        }
    } else {
        CARBON_CRITICAL("json parser: unexpected end of stream");
    }
}

JsonElement ReadJson(const std::string& jsonString) {
    const char* buf = jsonString.data();
    size_t pos = 0;
    size_t end = jsonString.size();
    JsonElement element = ReadJsonElement(buf, pos, end);
    SkipWhitespace(buf, pos, end);
    if (pos != end) {
        CARBON_CRITICAL(
            "json parser: expected end of input, but found additional text '{}'... at the end of the json file",
            std::string(buf + pos, std::min<size_t>(10, end - pos)));
    }
    return element;
}

inline void WriteTabs(std::ostream& s, int tabs) {
    for (int i = 0; i < tabs; ++i) {
        s << '\t';
    }
}

void WriteJsonElement(std::ostream& s, const JsonElement& jsonElement, int tabs) {
    switch (jsonElement.Type()) {
        case JsonElement::JsonType::Null: {
            s << "null";
        } break;
        case JsonElement::JsonType::False: {
            s << "false";
        } break;
        case JsonElement::JsonType::True: {
            s << "true";
        } break;
        case JsonElement::JsonType::Int: {
            s << epic::carbon::fmt::to_string(jsonElement.Value<int>());
        } break;
        case JsonElement::JsonType::Double: {
            s << epic::carbon::fmt::to_string(jsonElement.Value<double>());
        } break;
        case JsonElement::JsonType::String: {
            s << '\"' << jsonElement.String() << '\"';
        } break;
        case JsonElement::JsonType::Array: {
            s << '[';
            int counter = 0;
            for (const JsonElement& value : jsonElement.Array()) {
                if (counter++ != 0) {
                    s << ',';
                }
                if (tabs >= 0) {
                    s << '\n';
                    WriteTabs(s, tabs > 0 ? tabs + 1 : tabs);
                }
                WriteJsonElement(s, value, tabs > 0 ? tabs + 1 : tabs);
            }
            if ((tabs >= 0) && (jsonElement.Size() > 0)) {
                s << '\n';
                WriteTabs(s, tabs - 1);
            }
            s << ']';
        } break;
        case JsonElement::JsonType::Object: {
            s << '{';
            int counter = 0;
            for (const auto& [key, value] : jsonElement.Map()) {
                if (counter++ != 0) {
                    s << ',';
                }
                if (tabs >= 0) {
                    s << '\n';
                    WriteTabs(s, tabs);
                }
                s << '\"' << key << "\":";
                if (tabs >= 0) {
                    s << " ";
                }
                WriteJsonElement(s, value, tabs > 0 ? tabs + 1 : tabs);
            }
            if ((tabs >= 0) && (jsonElement.Size() > 0)) {
                s << '\n';
                WriteTabs(s, tabs - 1);
            }
            s << '}';
        } break;
    }
}

void WriteJson(std::ostream& stream, const JsonElement& jsonElement, int tabs) {
    WriteJsonElement(stream, jsonElement, tabs);
}

std::string WriteJson(const JsonElement& jsonElement, int tabs) {
    std::ostringstream s;
    WriteJson(s, jsonElement, tabs);
    return s.str();
}

}      // namespace carbon
}  // namespace epic
