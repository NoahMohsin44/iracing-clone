#pragma once
// Minimal JSON reader (objects, arrays, strings, numbers, booleans, null) for asset metadata.
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace apex {

class Json {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };

    static Json parse(const std::string& text, bool* ok = nullptr);

    Type type() const { return type_; }
    bool isNull() const { return type_ == Type::Null; }
    double number(double def = 0.0) const { return type_ == Type::Number ? num_ : def; }
    float f(float def = 0.0f) const { return static_cast<float>(number(def)); }
    int i(int def = 0) const { return static_cast<int>(number(def)); }
    bool boolean(bool def = false) const { return type_ == Type::Bool ? b_ : def; }
    const std::string& str() const { return str_; }
    const std::vector<Json>& array() const { return arr_; }
    size_t size() const { return type_ == Type::Array ? arr_.size() : obj_.size(); }
    const Json& operator[](const std::string& key) const;
    const Json& operator[](size_t idx) const;
    bool has(const std::string& key) const { return obj_.count(key) != 0; }

private:
    Type type_ = Type::Null;
    double num_ = 0.0;
    bool b_ = false;
    std::string str_;
    std::vector<Json> arr_;
    std::map<std::string, Json> obj_;
    friend struct JsonParser;
};

}  // namespace apex
