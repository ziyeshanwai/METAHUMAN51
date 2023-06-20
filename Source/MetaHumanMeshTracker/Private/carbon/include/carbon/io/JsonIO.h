// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/common/Common.h>

#include <map>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

namespace epic {
namespace carbon {

/**
 * JSON elements as defined in http://ietf.org/rfc/rfc4627.txt
 */
class JsonElement {
    private:
        // ! helper struct to implement partial specialization
        template<typename T>
        struct typeHelper {};

    public:
        // ! allowed types
        enum class JsonType {
            False,
            Null,
            True,
            Object,
            Array,
            Int,
            Double,
            String
        };

        JsonElement() : m_type(JsonType::Null), m_container() {
        }

        explicit JsonElement(JsonType type) : m_type(type) {
            if (IsArray()) {
                m_container = std::vector<JsonElement>();
            }
            if (IsObject()) {
                m_container = std::map<std::string, JsonElement>();
            }
        }

        explicit JsonElement(int value) : m_type(JsonType::Int), m_container(value) {
        }

        explicit JsonElement(double value) : m_type(JsonType::Double), m_container(value) {
        }

        explicit JsonElement(const std::string& str) : m_type(JsonType::String), m_container(str) {
        }

        explicit JsonElement(const char* str) : m_type(JsonType::String), m_container(str) {
        }

        explicit JsonElement(std::vector<JsonElement>&& array) : m_type(JsonType::Array), m_container(std::move(array)) {
        }

        explicit JsonElement(std::map<std::string, JsonElement>&& object) : m_type(JsonType::Object), m_container(std::move(
                                                                                                                      object)) {
        }

        explicit JsonElement(bool value) : m_type(value ? JsonType::True : JsonType::False) {
        }

        template<typename T>
        explicit JsonElement(const std::pair<T, T>& pair) : m_type(JsonType::Array) {
            m_container = std::vector<JsonElement>();
            Append(JsonElement(pair.first));
            Append(JsonElement(pair.second));
        }

        template<typename ... Types>
        explicit JsonElement(const std::tuple<Types ...>& tuple) : m_type(JsonType::Array) {
            m_container = std::vector<JsonElement>();
            std::apply([&](auto&&... arg) {
                    (Append(JsonElement(arg)), ...);
                }, tuple);
        }

        template<typename T>
        explicit JsonElement(const std::vector<T>& vec) : m_type(JsonType::Array) {
            m_container = std::vector<JsonElement>();
            for (const T& item : vec) {
                Append(JsonElement(item));
            }
        }

        template<typename T>
        explicit JsonElement(const std::map<std::string, T>& map) : m_type(JsonType::Object) {
            m_container = std::map<std::string, JsonElement>();
            for (const auto& [key, item] : map) {
                Insert(key, JsonElement(item));
            }
        }

        JsonElement(const JsonElement&) = delete;
        JsonElement& operator=(const JsonElement&) = delete;

        JsonElement(JsonElement&& other) : m_type(other.m_type), m_container(std::move(other.m_container)) {
            other.m_type = JsonType::Null;
        }

        JsonElement& operator=(JsonElement&& other) {
            if (&other != this) {
                m_type = other.m_type;
                m_container = std::move(other.m_container);
                other.m_type = JsonType::Null;
            }
            return *this;
        }

        JsonType Type() const {
            return m_type;
        }

        bool IsNull() const {
            return (Type() == JsonType::Null);
        }

        bool IsTrue() const {
            return (Type() == JsonType::True);
        }

        bool IsFalse() const {
            return (Type() == JsonType::False);
        }

        bool IsInt() const {
            return (Type() == JsonType::Int);
        }

        bool IsDouble() const {
            return (Type() == JsonType::Double);
        }

        bool IsNumber() const {
            return IsInt() || IsDouble();
        }

        bool IsString() const {
            return Type() == JsonType::String;
        }

        bool IsArray() const {
            return Type() == JsonType::Array;
        }

        bool IsObject() const {
            return Type() == JsonType::Object;
        }

        template<class T, typename DISCARD = typename std::enable_if<std::is_arithmetic<T>::value>::type>
        T Value() const {
            return IsInt() ? T(std::get<int>(m_container)) : T(std::get<double>(m_container));
        }

        const std::string& String() const {
            return std::get<std::string>(m_container);
        }

        const std::vector<JsonElement>& Array() const {
            return std::get<std::vector<JsonElement> >(m_container);
        }

        const std::map<std::string, JsonElement>& Object() const {
            return std::get<std::map<std::string, JsonElement> >(m_container);
        }

        bool Boolean() const {
            if (IsTrue()) {
                return true;
            } else if (IsFalse()) {
                return false;
            }
            CARBON_CRITICAL("json element does not represent a boolean value");
        }

        // ! Generic get function which calls the specializations using typeHelper.
        template<class T>
        T Get() const {
            return Get(typeHelper<T>());
        }

        size_t Size() const {
            if (IsArray()) {
                return Array().size();
            }
            if (IsObject()) {
                return Object().size();
            }
            CARBON_CRITICAL("only array and object have a size");
        }

        const JsonElement& operator[](size_t index) const {
            if (!IsArray()) {
                CARBON_CRITICAL("cannot index into json element as it is not an array");
            }
            if (index >= Array().size()) {
                CARBON_CRITICAL("index out of bounds");
            }
            return Array()[index];
        }

        void Append(JsonElement&& element) {
            if (!IsArray()) {
                CARBON_CRITICAL("can only append elements to an array");
            }
            MutableArray().emplace_back(std::move(element));
        }

        bool Contains(const std::string& key) const {
            return (Object().find(key) != Object().end());
        }

        const JsonElement& operator[](const std::string& key) const {
            if (!IsObject()) {
                CARBON_CRITICAL("cannot use key \"{}\" for json element as it is not an object", key);
            }
            auto it = Object().find(key);
            if (it != Object().end()) {
                return it->second;
            } else {
                CARBON_CRITICAL("no element with key \"{}\" in json object", key);
            }
        }

        const std::map<std::string, JsonElement>& Map() const {
            return Object();
        }

        void Insert(const std::string& key, JsonElement&& element) {
            if (!IsObject()) {
                CARBON_CRITICAL("can only insert elements into an array");
            }
            MutableObject().emplace(key, std::move(element));
        }

    private:
        // ! Generic Get function. B
        template<typename T>
        T Get(typeHelper<T>) const;

        // ! Partial specialization for std::pair.
        template<typename S, typename T>
        std::pair<S, T> Get(typeHelper<std::pair<S, T> >) const {
            if (!IsArray()) {
                CARBON_CRITICAL("json element is not an array");
            }
            if (Size() != 2) {
                CARBON_CRITICAL("json element is not an array of size 2");
            }
            return {Array()[0].Get<S>(), Array()[1].Get<T>()};
        }

        // ! Partial specialization for std::tuple.
        template<typename ... Types>
        std::tuple<Types ...> Get(typeHelper<std::tuple<Types ...> >) const {
            if (!IsArray()) {
                CARBON_CRITICAL("json element is not an array");
            }
            if (Size() != std::tuple_size<std::tuple<Types ...> >::value) {
                CARBON_CRITICAL("json element is not an array of size {}", std::tuple_size<std::tuple<Types ...> >::value);
            }

            std::tuple<Types ...> tupleOut;
            std::apply([&](auto&&... arg) {
                    size_t i(0);
                    ((arg = Array()[i++].Get<typename std::remove_reference<decltype(arg)>::type>()), ...);
                }, tupleOut);
            return tupleOut;
        }

        // ! Partial specialization for std::vector.
        template<typename S>
        std::vector<S> Get(typeHelper<std::vector<S> >) const {
            if (!IsArray()) {
                CARBON_CRITICAL("json element is not an array");
            }
            std::vector<S> out;
            out.reserve(Size());
            for (const JsonElement& element : Array()) {
                out.push_back(element.template Get<S>());
            }
            return out;
        }

        // ! Partial specialization for std::map.
        template<typename S>
        std::map<std::string, S> Get(typeHelper<std::map<std::string, S> >) const {
            if (!IsObject()) {
                CARBON_CRITICAL("json element is not an object");
            }
            std::map<std::string, S> out;
            for (const auto& [keyname, element] : Map()) {
                out.emplace(keyname, element.template Get<S>());
            }
            return out;
        }

    private:
        std::vector<JsonElement>& MutableArray() {
            return std::get<std::vector<JsonElement> >(m_container);
        }

        std::map<std::string, JsonElement>& MutableObject() {
            return std::get<std::map<std::string, JsonElement> >(m_container);
        }

    private:
        JsonType m_type;
        std::variant<int, double, std::string, std::vector<JsonElement>, std::map<std::string, JsonElement> > m_container;
};

// ! Get spezializations
template<> inline int JsonElement::Get<int>(typeHelper<int>) const {
    return Value<int>();
}

template<> inline size_t JsonElement::Get<size_t>(typeHelper<size_t>) const {
    return Value<size_t>();
}

template<> inline float JsonElement::Get<float>(typeHelper<float>) const {
    return Value<float>();
}

template<> inline double JsonElement::Get<double>(typeHelper<double>) const {
    return Value<double>();
}

template<> inline std::string JsonElement::Get<std::string>(typeHelper<std::string>) const {
    return String();
}

JsonElement ReadJson(const std::string& jsonString);

// ! Write @p json to string. Set @p tabs to 0 or positive to pretty print with tabs and newlines.
std::string WriteJson(const JsonElement& json, int tabs = -1);

// ! Write @p json to @p stream. Set @p tabs to 0 or positive to pretty print with tabs and newlines.
void WriteJson(std::ostream& stream, const JsonElement& element, int tabs = -1);

}  // namespace carbon
}  // namespace epic
