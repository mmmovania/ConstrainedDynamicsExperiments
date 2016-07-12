class BoxShape { 
  public float width, height, mass, momentOfInertia; 

  // Calculates the inertia of a box shape and stores it in the momentOfInertia variable.
  public void CalculateBoxInertia() {
    float m = mass;
    float w = width;
    float h = height;
    momentOfInertia = m * (w * w + h * h) / 12;
  }
};

// Two dimensional rigid body
class RigidBody {
    public PVector position, linearVelocity, force;
    public float angle, angularVelocity, torque;
    public BoxShape shape;
    
    // Applies a force at a point in the body, inducing some torque.
    void ComputeForceAndTorque() {
        PVector f = new PVector(0, 100);
        force = f;
        // r is the 'arm vector' that goes from the center of mass to the point of force application
        PVector r = new PVector(shape.width / 2, shape.height / 2);
        torque = r.x * f.y - r.y * f.x;
    }
} ;


final int NUM_RIGID_BODIES = 1;
final float totalSimulationTime = 10; // The simulation will run for 10 seconds.
float currentTime = 0; // This accumulates the time that has passed.
final float dt = 1; // Each step will take one second.
    
// Global array of rigid bodies.
RigidBody rigidBodies[];

// Prints the position and angle of each body on the output.
// We could instead draw them on screen.
void PrintRigidBodies() {
    for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
        RigidBody r = rigidBodies[i];
        System.out.print("body["+ i +"] p = ("+r.position.x+", "+r.position.y+"), a = "+ r.angle);
    }
}

// Initializes rigid bodies with random positions and angles and zero linear and angular velocities.
// They're all initialized with a box shape of random dimensions.
void InitializeRigidBodies() {
    for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
        rigidBodies[i] = new RigidBody();
        rigidBodies[i].position = new PVector(random(50), random(50));
        
        rigidBodies[i].linearVelocity = new PVector(0, 0);
        rigidBodies[i].angularVelocity = 0;
        
        BoxShape shape=new BoxShape();
        shape.mass = 10;
        shape.width = 1 + random(2);
        shape.height = 1 + random(2);
        shape.CalculateBoxInertia();
        rigidBodies[i].shape = shape;
    }
}



void RunRigidBodySimulation() {
   
  System.out.println("Simulation Started");
    while (currentTime < totalSimulationTime) {
        //sleep(dt);
        
        for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
            RigidBody rigidBody = rigidBodies[i];
            rigidBody.ComputeForceAndTorque();
            PVector linearAcceleration = new PVector(rigidBody.force.x / rigidBody.shape.mass, rigidBody.force.y / rigidBody.shape.mass);
            rigidBody.linearVelocity.x += linearAcceleration.x * dt;
            rigidBody.linearVelocity.y += linearAcceleration.y * dt;
            rigidBody.position.x += rigidBody.linearVelocity.x * dt;
            rigidBody.position.y += rigidBody.linearVelocity.y * dt;
            float angularAcceleration = rigidBody.torque / rigidBody.shape.momentOfInertia;
            rigidBody.angularVelocity += angularAcceleration * dt;
            rigidBody.angle += rigidBody.angularVelocity * dt;
        }
               
        PrintRigidBodies();
        System.out.println(" Time: "+currentTime);
        
        currentTime += dt;
    }
    System.out.println("Simulation Ends");
    exit();
}
    
    
void setup() {
  rigidBodies = new RigidBody[NUM_RIGID_BODIES];
  InitializeRigidBodies(); 
}
 
 
 

void draw() {
  RunRigidBodySimulation();
}
