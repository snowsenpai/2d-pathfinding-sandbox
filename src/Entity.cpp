#include "Entity.h"

#include <string>

Entity::Entity(const size_t& id, const std::string& tag)
	: m_tag(tag)
	, m_id(id)
{}

size_t Entity::getId() const
{
	return m_id;
}

bool Entity::isActive() const
{
	return m_active;
}

const std::string& Entity::getTag() const
{
	return m_tag;
}

void Entity::destroy()
{
	m_active = false;
}