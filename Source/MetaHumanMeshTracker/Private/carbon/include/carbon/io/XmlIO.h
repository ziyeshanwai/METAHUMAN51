// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/common/Common.h>

#include <iomanip>
#include <map>
#include <ostream>
#include <string>

namespace epic {
namespace carbon {
namespace xml {

/**
 * XML element with name, attributes, text, and xml element children.
 */
class XMLElement {
    public:
        // ! Creates an XMLElement with name @p name
        XMLElement(const std::string& name) : m_name(name) {
        }

        // ! Add a child element and return reference to it.
        XMLElement& AddChild(const std::string& childName) {
            m_children.emplace_back(std::make_unique<XMLElement>(childName));
            return *m_children.back();
        }

        const std::string& Name() const {
            return m_name;
        }

        void SetText(const std::string& text) {
            m_text = text;
        }

        const std::string& Text() const {
            return m_text;
        }

        void AddAttribute(const std::string& attributeName, const std::string& attributeValue) {
            m_attributes[attributeName] = attributeValue;
        }

        const std::string& Attribute(const std::string& attributeName) const {
            auto it = m_attributes.find(attributeName);
            if (it != m_attributes.end()) {
                return it->second;
            } else {
                CARBON_CRITICAL("no attribute {} in element {}", attributeName, m_name);
            }
        }

        const std::map<std::string, std::string>& Attributes() const {
            return m_attributes;
        }

        const std::vector<std::unique_ptr<XMLElement> >& Children() const {
            return m_children;
        }

        // ! @returns all children with element name @p childName
        std::vector<XMLElement*> ChildrenWithName(const std::string& childName) {
            std::vector<XMLElement*> elements;
            for (const auto& child : m_children) {
                if (child->Name() == childName) {
                    elements.push_back(child.get());
                }
            }
            return elements;
        }

        /**
         * @returns a unique child element @p childName.
         * @param[in] childName   The name of the child.
         * @param[in] failIfNotUnique  If True then the method throws an exception is there is no child or multiple children with the name, otherwise it will return a nullptr.
         */
        template<bool failIfNotUnique = true>
        XMLElement* UniqueChild(const std::string& childName) {
            std::vector<XMLElement*> allChildren = ChildrenWithName(childName);
            if (allChildren.size() == 1) {
                return allChildren[0];
            }
            if constexpr (failIfNotUnique) {
                CARBON_CRITICAL("{} is not a single child of {}", childName, m_name);
            } else {
                return nullptr;
            }
        }

    private:
        std::string m_name;
        std::map<std::string, std::string> m_attributes;
        std::string m_text;
        // ! Store children as unique_ptr to handle any potential vector resizing which would break the references
        std::vector<std::unique_ptr<XMLElement> > m_children;
};

// ! Writes the XML element to @p output
void WriteXML(std::ostream& out, const XMLElement& element);

// ! Writes the XML element to std::string.
std::string WriteXML(const XMLElement& element);

// ! Reads XML data from @p xmlData and @returns it as XMLElemen.
XMLElement ReadXML(const std::string& xmlData);

}  // namespace xml
}  // namespace carbon
}  // namespace epic
