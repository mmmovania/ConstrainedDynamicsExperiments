class Particle {
  public PVector position, velocity;
  float mass;
};

final int NUM_PARTICLES = 1;
Particle particles[];
final float totalSimulationTime = 10; // The simulation will run for 10 seconds.
float currentTime = 0; // This accumulates the time that has passed.
final float dt = 1; // Each step will take one second.
    
    
    
void setup() {
  particles = new Particle[NUM_PARTICLES];
  InitializeParticles();
  PrintParticles(); 
}

// Prints all particles' position to the output. We could instead draw them on screen
// in a more interesting application.
void  PrintParticles() {
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        Particle p = particles[i];
        System.out.print("particle["+ i +"] ("+p.position.x+", "+p.position.y+")");
    }
}

// Initializes all particles with random positions, zero velocities and 1kg mass.
void InitializeParticles() {
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i]=new Particle();
        particles[i].position = new PVector(random(50), random(50));
        particles[i].velocity = new PVector(0,0);
        particles[i].mass = 1;
    }
}

// Just applies Earth's gravity force (mass times gravity acceleration 9.81 m/s^2) to each particle.
PVector ComputeForce(Particle p) {
    return new PVector(0, p.mass * -9.81);
}

void RunSimulation() {
    
    System.out.println("Simulation Started");
    while (currentTime < totalSimulationTime) {
        // We're sleeping here to keep things simple. In real applications you'd use some
        // timing API to get the current time in milliseconds and compute dt in the beginning 
        // of every iteration like this:
        // currentTime = GetTime()
        // dt = currentTime - previousTime
        // previousTime = currentTime
        //sleep(dt);

        for (int i = 0; i < NUM_PARTICLES; ++i) {
            Particle particle = particles[i];
            PVector force = ComputeForce(particle);
            PVector acceleration = new PVector(force.x / particle.mass, force.y / particle.mass);
            particle.velocity.x += acceleration.x * dt;
            particle.velocity.y += acceleration.y * dt;
            particle.position.x += particle.velocity.x * dt;
            particle.position.y += particle.velocity.y * dt;
        }
        
        PrintParticles();
        System.out.println(" Time: "+currentTime);
        currentTime += dt;
    }
    System.out.println("Simulation Ends");
    exit();
}

void draw() {
  RunSimulation();
}
