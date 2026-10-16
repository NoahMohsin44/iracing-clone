#include "core/Json.h"

#include <cstdlib>

namespace apex {

struct JsonParser {
    const std::string& s;
    size_t p = 0;
    bool ok = true;

    void ws() {
        while (p < s.size() && (s[p] == ' ' || s[p] == '\n' || s[p] == '\r' || s[p] == '\t')) ++p;
    }

    static void appendUtf8(std::string& out, unsigned cp) {
        if (cp < 0x80) {
            out += static_cast<char>(cp);
        } else if (cp < 0x800) {
            out += static_cast<char>(0xC0 | (cp >> 6));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else {
            out += static_cast<char>(0xE0 | (cp >> 12));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        }
    }

    std::string parseString() {
        std::string out;
        ++p;  // opening quote
        while (p < s.size() && s[p] != '"') {
            char c = s[p++];
            if (c == '\\' && p < s.size()) {
                char e = s[p++];
                switch (e) {
                    case 'n': out += '\n'; break;
                    case 't': out += '\t'; break;
                    case 'r': out += '\r'; break;
                    case 'b': out += '\b'; break;
                    case 'f': out += '\f'; break;
                    case 'u': {
                        unsigned cp = static_cast<unsigned>(strtoul(s.substr(p, 4).c_str(), nullptr, 16));
                        p += 4;
                        appendUtf8(out, cp);
                        break;
                    }
                    default: out += e; break;
                }
            } else {
                out += c;
            }
        }
        ++p;  // closing quote
        return out;
    }

    Json value() {
        ws();
        Json j;
        if (p >= s.size()) {
            ok = false;
            return j;
        }
        char c = s[p];
        if (c == '{') {
            j.type_ = Json::Type::Object;
            ++p;
            ws();
            if (p < s.size() && s[p] == '}') {
                ++p;
                return j;
            }
            while (ok && p < s.size()) {
                ws();
                if (s[p] != '"') {
                    ok = false;
                    break;
                }
                std::string key = parseString();
                ws();
                if (p >= s.size() || s[p] != ':') {
                    ok = false;
                    break;
                }
                ++p;
                j.obj_[key] = value();
                ws();
                if (p < s.size() && s[p] == ',') {
                    ++p;
                    continue;
                }
                if (p < s.size() && s[p] == '}') {
                    ++p;
                    break;
                }
                ok = false;
            }
        } else if (c == '[') {
            j.type_ = Json::Type::Array;
            ++p;
            ws();
            if (p < s.size() && s[p] == ']') {
                ++p;
                return j;
            }
            while (ok && p < s.size()) {
                j.arr_.push_back(value());
                ws();
                if (p < s.size() && s[p] == ',') {
                    ++p;
                    continue;
                }
                if (p < s.size() && s[p] == ']') {
                    ++p;
                    break;
                }
                ok = false;
            }
        } else if (c == '"') {
            j.type_ = Json::Type::String;
            j.str_ = parseString();
        } else if (s.compare(p, 4, "true") == 0) {
            j.type_ = Json::Type::Bool;
            j.b_ = true;
            p += 4;
        } else if (s.compare(p, 5, "false") == 0) {
            j.type_ = Json::Type::Bool;
            p += 5;
        } else if (s.compare(p, 4, "null") == 0) {
            p += 4;
        } else {
            char* end = nullptr;
            j.num_ = strtod(s.c_str() + p, &end);
            if (end == s.c_str() + p) {
                ok = false;
            } else {
                j.type_ = Json::Type::Number;
                p = static_cast<size_t>(end - s.c_str());
            }
        }
        return j;
    }
};

Json Json::parse(const std::string& text, bool* ok) {
    JsonParser parser{text};
    Json j = parser.value();
    if (ok) *ok = parser.ok;
    return j;
}

const Json& Json::operator[](const std::string& key) const {
    static const Json kNull;
    auto it = obj_.find(key);
    return it == obj_.end() ? kNull : it->second;
}

const Json& Json::operator[](size_t idx) const {
    static const Json kNull;
    return idx < arr_.size() ? arr_[idx] : kNull;
}

}  // namespace apex
