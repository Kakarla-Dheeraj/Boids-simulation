#include <SFML/Graphics.hpp>
#include <vector>      
#include <random>      
#include <optional>    
#include <cstdint>    

class ParticleSystem : public sf::Drawable, public sf::Transformable
{
public:
    ParticleSystem(unsigned int count=100) : m_particles(count)
    {
        for (std::size_t i = 0; i < m_particles.size(); ++i)
        {
            createParticle(i);
        }
    }

    void update(sf::Time elapsed)
    {
        for(auto& p : m_particles)
        {
            p.steering = {0.f, 0.f};
        }

        for (std::size_t i = 0; i < m_particles.size(); ++i)
        {
            Particle& p = m_particles[i];

            //Check if the particle collides with the wall
            edgeSteering(p);

            //Seperation rule
            boidRules(i);

            //Add tiny noise
            addAngularNoise(i);

        }
        
        for(auto& p : m_particles)
        {
            p.velocity += p.steering;

            limitSpeed(p);

            sf::Angle angle =  sf::radians(std::atan2(p.velocity.y, p.velocity.x));

            angle += sf::degrees(90);

            p.triangle.setRotation(angle);

            p.position += p.velocity * elapsed.asSeconds();

            p.triangle.setPosition(p.position);
        }
    }

private:

    struct Particle
    {
        sf::Vector2f position;
        sf::Vector2f velocity;
        sf::Vector2f steering;
        float noiseAngle = 0.f;
        sf::CircleShape triangle{10.f,3};

    };

    void draw(sf::RenderTarget& target, sf::RenderStates states) const override
    {
        // apply the transform
        states.transform *= getTransform();

        // our particles don't use a texture
        states.texture = nullptr;

        // draw the vertex array
        for(int i=0;i<m_particles.size();i++){
            target.draw(m_particles[i].triangle, states);
        }
        
    }

    void createParticle(int index)
    {
        // create random number generator
        static std::random_device rd;
        static std::mt19937       rng(rd());

        // give a random velocity and lifetime to the particle
        const float     speed_x       = std::uniform_real_distribution(-200.f, 200.f)(rng);
        const float     speed_y       = std::uniform_real_distribution(-200.f, 200.f)(rng);
        m_particles[index].velocity = sf::Vector2f(speed_x, speed_y);
        const float position_x = std::uniform_real_distribution(0.f, 1920.f)(rng);
        const float position_y = std::uniform_real_distribution(0.f, 1080.f)(rng);

        m_particles[index].position = sf::Vector2f(position_x, position_y);
        m_particles[index].triangle.setPosition(m_particles[index].position);
        sf::Angle inverse =sf::radians(std::atan2(speed_y,speed_x));
        inverse+=sf::degrees(90);
        m_particles[index].triangle.setRotation(inverse);
    }

    void edgeSteering(Particle& p)
    {
        const float margin = 100.f;
        const float turnFactor = 2.f;

        if (p.position.x < margin)
            p.steering.x += turnFactor;

        if (p.position.x > 1920 - margin)
            p.steering.x -= turnFactor;

        if (p.position.y < margin)
            p.steering.y += turnFactor;

        if (p.position.y > 1080 - margin)
            p.steering.y -= turnFactor;
    }
    void limitSpeed(Particle& p)
    {
        float speed =
            std::sqrt(
                p.velocity.x * p.velocity.x +
                p.velocity.y * p.velocity.y);

        if(speed < minSpeed)
        {
            p.velocity =
                (p.velocity / speed) * minSpeed;
        }

        if(speed > maxSpeed)
        {
            p.velocity =
                (p.velocity / speed) * maxSpeed;
        }
    }

    void boidRules(int index)
    {
        float xpos_avg = 0;
        float ypos_avg = 0;
        float xvel_avg = 0;
        float yvel_avg = 0;

        float close_dx = 0;
        float close_dy = 0;

        int neighboring_boids = 0;

        Particle& boid = m_particles[index];

        for(int i = 0; i < m_particles.size(); i++)
        {
            if(i == index)
                continue;

            float dx = boid.position.x - m_particles[i].position.x;
            float dy = boid.position.y - m_particles[i].position.y;

            float sqr_distance = dx * dx + dy * dy;

            if(sqr_distance < protectedRange * protectedRange)
            {
                close_dx += dx;
                close_dy += dy;
            }
            else if(sqr_distance < visualRange * visualRange)
            {
                xpos_avg += m_particles[i].position.x;
                ypos_avg += m_particles[i].position.y;

                xvel_avg += m_particles[i].velocity.x;
                yvel_avg += m_particles[i].velocity.y;

                neighboring_boids++;
            }
        }

        if(neighboring_boids > 0)
        {
            xpos_avg /= neighboring_boids;
            ypos_avg /= neighboring_boids;

            xvel_avg /= neighboring_boids;
            yvel_avg /= neighboring_boids;

            boid.steering.x +=
                (xpos_avg - boid.position.x) * centeringFactor;

            boid.steering.y +=
                (ypos_avg - boid.position.y) * centeringFactor;

            boid.steering.x +=
                (xvel_avg - boid.velocity.x) * matchingFactor;

            boid.steering.y +=
                (yvel_avg - boid.velocity.y) * matchingFactor;
        }

        boid.steering.x += close_dx * avoidFactor;
        boid.steering.y += close_dy * avoidFactor;
    }

    void addAngularNoise(int index)
    {
        static std::random_device rd;
        static std::mt19937 rng(rd());

        Particle& p = m_particles[index];

        p.noiseAngle +=
            std::uniform_real_distribution<float>(-0.05f, 0.05f)(rng);

        float heading =
            std::atan2(p.velocity.y, p.velocity.x);

        float noisyHeading =
            heading + p.noiseAngle;

        p.steering.x += std::cos(noisyHeading) * noiseFactor;
        p.steering.y += std::sin(noisyHeading) * noiseFactor;
    }

    std::vector<Particle> m_particles;

    float protectedRange = 50.f;
    float visualRange = 75.f;

    float avoidFactor = 0.1f;
    float matchingFactor = 0.01f;
    float centeringFactor = 0.00005f;
    float noiseFactor=0.2f;

    float minSpeed = 100.f;
    float maxSpeed = 250.f;
};

int main()
{
    //Setting anti-aliasing
    sf::ContextSettings settings;
    settings.antiAliasingLevel = 8;

    // create the window
    sf::RenderWindow window(sf::VideoMode({1920, 1080}), "Boids", sf::Style::Default, sf::State::Windowed, settings);

    // create the particle system
    ParticleSystem particles;

    // create a clock to track the elapsed time
    sf::Clock clock;

    // run the main loop
    while (window.isOpen())
    {
        // handle events
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        sf::Time elapsed = clock.restart();
        particles.update(elapsed);

        // draw it
        window.clear();
        window.draw(particles);
        window.display();
    }
}