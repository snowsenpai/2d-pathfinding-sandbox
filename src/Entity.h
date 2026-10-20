#pragma once

#include "Components.h"

#include <tuple>
#include <string>
#include <utility>
#include <optional>

class EntityManager;

using ComponentTuple = std::tuple<
	std::optional<CTransform>,
	std::optional<CBoundingBox>,
	std::optional<CBoundingCircle>,
	std::optional<CShape>,
	std::optional<CInput>
>;

class Entity
{
	friend class EntityManager;

	bool				m_active = true;
	size_t				m_id = 0;
	std::string			m_tag = "default";
	ComponentTuple		m_components;

	// constructor is private so we can never create
	// entities outside the EntityManager which has friend access
	Entity(const size_t& id, const std::string& tag);

public:
	void				destroy();
	size_t				getId()		const;
	bool				isActive()	const;
	const std::string& getTag()	const;

	template<typename T>
	bool hasComponent() const
	{
		return getComponent<T>().has_value();
	}

	/**
	 * Adds or replaces the component of type T on this entity.
	 * If a component of this type already exists, it will be overwritten.
	 * This is intentional — systems such as the animation system may swap
	 * components at runtime (e.g. switching CAnimation based on player state).
	 *
	 * @tparam T      The component type to add/replace
	 * @tparam TArgs  Constructor argument types for T
	 * @param  mArgs  Arguments forwarded to T's constructor
	 * @return        A reference to the newly constructed component
	 */
	template<typename T, typename... TArgs>
	T& addComponent(TArgs&&... mArgs)
	{
		auto& component = getComponent<T>();
		component.emplace(std::forward<TArgs>(mArgs)...);
		return component.value();
	}

	template<typename T>
	std::optional<T>& getComponent()
	{
		return std::get<std::optional<T>>(m_components);
	}

	template<typename T>
	const std::optional<T>& getComponent() const
	{
		return std::get<std::optional<T>>(m_components);
	}

	template<typename T>
	void removeComponent()
	{
		getComponent<T>() = std::nullopt;
	}
};