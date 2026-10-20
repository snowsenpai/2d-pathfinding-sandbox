#include "Game.h"
#include "Vec2.h"
#include "Components.h"

#include <SFML/Graphics.hpp>

#include <memory>

void Game::run()
{
	init();

	while (m_running)
	{
		update();
	}
	m_window.close();
}

void Game::update()
{
	m_entityManager.update();

	sPlayerInput();
	sMovement();
	sCollision();
	sRender();
}

void Game::quit()
{
	m_running = false;
}

void Game::init()
{
	m_window.create(sf::VideoMode({ 800, 600 }), "Magic!");
	m_window.setFramerateLimit(60);

	spawnPlayer();
	m_running = true;
}

void Game::spawnPlayer()
{
	auto player = m_entityManager.addEntity("player");
	
	float playerRadius = 40.f;
	int shapePoints = 10;
	sf::Color fillColor(sf::Color::Transparent);
	sf::Color outlineColor(sf::Color::Red);
	float thickness = 3.f;

	player->addComponent<CShape>(playerRadius, shapePoints, fillColor, outlineColor, thickness);
	
	// spawn player at a random position within the window
	float maxPosX = static_cast<float>(m_window.getSize().x - playerRadius);
	float maxPosY = static_cast<float>(m_window.getSize().y - playerRadius);
	
	float posX = rng(playerRadius, maxPosX);
	float posY = rng(playerRadius, maxPosY);

	player->addComponent<CTransform>(Vec2(posX, posY));

	float boxSize = playerRadius * 2;
	player->addComponent<CBoundingBox>(Vec2(boxSize, boxSize));
	
	player->addComponent<CInput>();
	
	m_player = player;
}

void Game::sRender()
{
	m_window.clear();

	for (auto& e : m_entityManager.getEntities())
	{
		if (!e->hasComponent<CTransform>() || !e->hasComponent<CShape>()) continue;

		auto& entityCircle = e->getComponent<CShape>().value().circle;
		auto& entityPos = e->getComponent<CTransform>().value().pos;
		
		entityCircle.setPosition({ entityPos.x, entityPos.y });

		m_window.draw(entityCircle);
		
		if (m_drawCollisison)
		{
			if (e->hasComponent<CBoundingBox>())
			{
				auto& eBox = e->getComponent<CBoundingBox>().value().size;
			
				sf::RectangleShape box({ eBox.x, eBox.y });

				box.setOrigin({ eBox.x / 2.f, eBox.y / 2.f });
				box.setPosition({ entityPos.x, entityPos.y });
				box.setOutlineColor(sf::Color::Green);
				box.setFillColor(sf::Color::Transparent);
				box.setOutlineThickness(1.f);
			
				m_window.draw(box);
			}
			else if (e->hasComponent<CBoundingCircle>())
			{
				auto& eCircle = e->getComponent<CBoundingCircle>().value();

				sf::CircleShape circle(eCircle.radius);

				circle.setOrigin({ eCircle.radius, eCircle.radius });
				circle.setPosition({ entityPos.x, entityPos.y });
				circle.setOutlineColor(sf::Color::Green);
				circle.setFillColor(sf::Color::Transparent);
				circle.setOutlineThickness(1.f);

				m_window.draw(circle);
			}
		}
	}

	m_window.display();
}

void Game::sCollision()
{
	// player circle x window collision (in this case, player's bounding box or circle is not relevant for window collision)
	auto windowSize = m_window.getView().getSize();

	float playerRadius = m_player->getComponent<CShape>().value().circle.getRadius();
	
	auto& playerTransform = m_player->getComponent<CTransform>().value();
	
	// x axis
	if (
		// left
		playerTransform.pos.x - playerRadius <= 0.0f ||
		// right
		playerTransform.pos.x + playerRadius >= static_cast<float>(windowSize.x)
		)
	{
		playerTransform.pos.x = playerTransform.prevPos.x;
	}

	// y axis
	if (
		// bottom
		playerTransform.pos.y + playerRadius >= static_cast<float>(windowSize.y) ||
		// top
		playerTransform.pos.y - playerRadius <= 0.0f
		)
	{
		playerTransform.pos.y = playerTransform.prevPos.y;
	}
}

void Game::sMovement()
{
	auto& pTransform = m_player->getComponent<CTransform>().value();
	auto& pInput = m_player->getComponent<CInput>().value();

	// reset player velocity on each frame
	pTransform.velocity = { 0.0f, 0.0f };

	float speed = 8.f;
	if (pInput.up)
	{
		pTransform.velocity.y = -speed;
	}
	else if (pInput.down)
	{
		pTransform.velocity.y = speed;
	}
	else if (pInput.left)
	{
		pTransform.velocity.x = -speed;
	}
	else if (pInput.right)
	{
		pTransform.velocity.x = speed;
	}

	// update players position
	pTransform.prevPos = pTransform.pos;
	pTransform.pos += pTransform.velocity;
}

void Game::sPlayerInput()
{
	while (const std::optional event = m_window.pollEvent())
	{
		if (event->is<sf::Event::Closed>())
		{
			quit();
		}

		if (const auto* windowResized = event->getIf<sf::Event::Resized>())
		{
			sf::FloatRect visibleArea({ 0.0f, 0.0f }, { static_cast<float>(windowResized->size.x), static_cast<float>(windowResized->size.y) });

			m_window.setView(sf::View(visibleArea));
		}

		if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>())
		{
			switch (keyPressed->code)
			{
			case sf::Keyboard::Key::W:
				m_player->getComponent<CInput>().value().up = true;
				break;
			case sf::Keyboard::Key::S:
				m_player->getComponent<CInput>().value().down = true;
				break;
			case sf::Keyboard::Key::A:
				m_player->getComponent<CInput>().value().left = true;
				break;
			case sf::Keyboard::Key::D:
				m_player->getComponent<CInput>().value().right = true;
				break;
			case sf::Keyboard::Key::C:
				m_drawCollisison = !m_drawCollisison;
				break;
			default:
				break;
			}
		}
		else if (const auto* keyReleased = event->getIf<sf::Event::KeyReleased>())
		{
			switch (keyReleased->code)
			{
			case sf::Keyboard::Key::W:
				m_player->getComponent<CInput>().value().up = false;
				break;
			case sf::Keyboard::Key::S:
				m_player->getComponent<CInput>().value().down = false;
				break;
			case sf::Keyboard::Key::A:
				m_player->getComponent<CInput>().value().left = false;
				break;
			case sf::Keyboard::Key::D:
				m_player->getComponent<CInput>().value().right = false;
				break;
			default:
				break;
			}
		}
	}
}