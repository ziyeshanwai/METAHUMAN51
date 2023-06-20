// Copyright Epic Games, Inc. All Rights Reserved.

#include <carbon/io/XmlIO.h>

#include <sstream>

namespace epic {
namespace carbon {
namespace xml {

static constexpr const char* TAB = "  ";

void WriteElement(std::ostream& out, const XMLElement& element, const std::string& tabSpace)
{
    // write element
    out << tabSpace << "<" << element.Name();
    // write attributes
    for (const auto&[key, value] : element.Attributes()) {
        out << " " << key << "=\"" << value << "\"";
    }
    if (element.Children().size() == 0 && element.Text().empty()) {
        // no children and no text, close element and go to next line
        out << "/>" << std::endl;
        return;
    }
    // write end of element
    out << ">";
    // write as one liner if there are no children present
    const bool oneLiner = (element.Children().size() == 0);
    if (!oneLiner) out << std::endl;
    // write text
    if (!element.Text().empty()) {
        if (!oneLiner) out << tabSpace;
        out << element.Text();
        if (!oneLiner) out << std::endl;
    }
    // write children
    for (const auto& child : element.Children()) {
        WriteElement(out, *child, tabSpace + TAB);
    }
    if (!oneLiner) out << tabSpace;
    // write end of element
    out << "</" << element.Name() << ">" << std::endl;
}

void WriteXML(std::ostream& out, const XMLElement& element)
{
    // write prolog
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl;
    WriteElement(out, element, "");
}

std::string WriteXML(const XMLElement& element)
{
    std::stringstream out;
    WriteXML(out, element);
    return out.str();
}

//! Check if a character is a whitespace
inline bool IsWhitespace(const char c)
{
    return (c == ' ' || c == '\t' || c == '\n' || c == '\v' || c == '\f' || c == '\r');
}

//! Skip any whitespace character.
inline void SkipWhitespace(const char* buf, size_t& pos, size_t end)
{
    while (pos < end && IsWhitespace(buf[pos])) {
        pos++;
    }
}

//! Verify that the next tocken is \p expectedToken1
void ExpectToken(const char* buf, size_t& pos, size_t end, const char expectedToken) {
    if (pos < end) {
        if (buf[pos] != expectedToken) {
            CARBON_CRITICAL("unexpected token: {} instead of {}", buf[pos], expectedToken);
        }
        pos++;
    } else {
        CARBON_CRITICAL("end of stream, but expected token {}", expectedToken);
    }
}

//! Parse the next name until encoutering a whitespace or any charactor of '>/='
std::string ParseName(const char* buf, size_t& pos, size_t end)
{
    size_t startPos = pos;
    while (pos < end) {
        const char c = buf[pos];
        if (!IsWhitespace(c) && c != '>' && c != '/' && c != '=') {
            pos++;
        } else {
            break;
        }
    }
    return std::string(buf + startPos, pos - startPos);
}

void ReadXMLElement(XMLElement& element, const std::string& xmlData, size_t& pos, const size_t end)
{
    const char* buf = xmlData.data();
    // LOG_INFO("element name: {}", element.name);
    SkipWhitespace(buf, pos, end);
    // attributes or end of element or end of total element?
    if (pos < end) {
        while (true) {
            // process attributes
            if (buf[pos] == '/' || buf[pos] == '>') break;
            const std::string attributeName = ParseName(buf, pos, end);
            pos++; // go to element after '='
            SkipWhitespace(buf, pos, end);
            const char delim = buf[pos];
            pos++;
            size_t prevPos = pos;
            pos = xmlData.find(delim, pos);
            const std::string attributeValue = xmlData.substr(prevPos, pos - prevPos);
            pos++;
            element.AddAttribute(attributeName, attributeValue);
            // LOG_INFO("adding attribute {}=\"{}\" to element \"{}\"", attributeName, attributeValue, element.name);
            SkipWhitespace(buf, pos, end);
        }
        if (buf[pos] == '/') {
            // end of xml element, hence no text and no children
            pos++;
            ExpectToken(buf, pos, end, '>');
        }
        else if (buf[pos] == '>') {
            pos++;
            // end of xml element, proceed to process text and/or children
            while (true) {
                SkipWhitespace(xmlData.data(), pos, end);
                if (pos + 1 >= end) {
                    CARBON_CRITICAL("unexpected end of stream");
                }
                if (buf[pos] == '<') {
                    pos++;
                    if (buf[pos] == '/') {
                        // end of element
                        pos++;
                        const std::string endOfElementName = ParseName(buf, pos, end);
                        if (endOfElementName != element.Name()) {
                            CARBON_CRITICAL("end of element name is different from start: {} vs {}", element.Name(), endOfElementName);
                        }
                        SkipWhitespace(buf, pos, end);
                        ExpectToken(buf, pos, end, '>');
                        break;
                    } else {
                        // next element
                        const std::string childName = ParseName(buf, pos, end);
                        ReadXMLElement(element.AddChild(childName), xmlData, pos, end);
                    }
                } else {
                    // text
                    size_t prev_pos = pos;
                    pos = xmlData.find("<", pos);
                    element.SetText(element.Text() + xmlData.substr(prev_pos, pos - prev_pos));
                }
            }
        }
    }
}

XMLElement ReadXML(const std::string& xmlData)
{
    const char* buf = xmlData.data();
    size_t end = xmlData.length();
    size_t pos = 0;

    SkipWhitespace(xmlData.data(), pos, end);
    ExpectToken(buf, pos, end, '<');
    // parse name
    size_t pos_tmp = pos;
    std::string name = ParseName(buf, pos_tmp, end);
    if (name == "?xml") {
        // xml document starts with prolog, so let's find the end of it
        pos = xmlData.find("?>", pos);
        if (pos == std::string::npos) {
            CARBON_CRITICAL("failed to find end of xml prolog");
        }
        pos += 2;
        // proceed to the next token
        SkipWhitespace(xmlData.data(), pos, end);
        ExpectToken(buf, pos, end, '<');
    }

    name = ParseName(buf, pos, end);
    XMLElement element(name);
    ReadXMLElement(element, xmlData, pos, end);
    return element;
}

} // namespace xml
} // namespace carbon
} // namespace epic
