#include "EntityManager.h"
#include "Entity.h"

#include <memory>
#include <string>
#include <vector>
#include <algorithm>

void EntityManager::update()
{
	for (auto& e : m_entititesToAdd)
	{
		m_entities.emplace_back(e);
		m_entityMap[e->getTag()].emplace_back(e);
	}
	m_entititesToAdd.clear();

	removeDeadEntities(m_entities);

	for (auto& [_, entityVec] : m_entityMap)
	{
		removeDeadEntities(entityVec);
	}
}

std::shared_ptr<Entity> EntityManager::addEntity(const std::string& tag)
{
	auto newEntity = std::shared_ptr<Entity>(new Entity(m_totalEntities++, tag));
	m_entititesToAdd.push_back(newEntity);
	return newEntity;
}

EntityVec& EntityManager::getEntities()
{
	return m_entities;
}

EntityVec& EntityManager::getEntities(const std::string& entityGroup)
{
	return m_entityMap[entityGroup];
}

void EntityManager::removeDeadEntities(EntityVec& entities)
{
	auto newEnd = std::remove_if(
		entities.begin(),
		entities.end(),
		[](const std::shared_ptr<Entity>& e) {
			return !e->isActive();
		}
	);
	
	entities.erase(newEnd, entities.end());
}