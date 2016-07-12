class Particle {
  public PVector position, velocity;
  float mass;
};
 



Particle particle; 
final int radius = 200;
final float dt = 1/10.0;
final float angle = radians(0);
final float EPSILON = 0.0001;
final float ks = 0.5;
final float kd = 0.995;
PVector Fext, Fc, Ff, center;

void updateDistanceConstraint() {
  final float rest_length = radius;
  PVector dir = PVector.sub(center, particle.position);
  float len = dir.mag(); 
  if(len <= EPSILON) 
     return;
        
  float w1 = particle.mass;
  float w2 = particle.mass;
  float invMass = w1+ w2; 
  if(invMass <= EPSILON) 
     return;
 
  dir.normalize();
  PVector dP = PVector.mult(dir, (1.0f/invMass) * (len-rest_length ) * ks);
  if(w1 > 0.0)
     center = PVector.sub(center, PVector.mult(dP, w1));

  if(w2 > 0.0)
     particle.position = PVector.add(particle.position, PVector.mult(dP, w2) );   
}

void updateFeedbackForce() {
  final float rest_length = radius;
  PVector dir = PVector.sub(center, particle.position);
  float len = dir.mag(); 
  if(len <= EPSILON) 
     return;
        
  float w1 = particle.mass;
  float w2 = particle.mass;
  float invMass = w1+ w2; 
  if(invMass <= EPSILON) 
     return;
 
  dir.normalize();
  Ff = PVector.mult(dir, (1.0f/invMass) * (len-rest_length ) * ks);    
}

void updateFeedbackForce1() {
  PVector p1 = center;
  PVector p2 = particle.position;
  final float rest_length = radius;
  PVector v1 = new PVector(0,0);
  PVector v2 = particle.velocity;
  
  PVector deltaP = PVector.sub(p1,p2);
  PVector deltaV = PVector.sub(v1,v2);
  float dist = deltaP.mag();

  float leftTerm = -ks * (dist - rest_length);
  float rightTerm = kd * (PVector.dot(deltaV, deltaP) / dist);
  deltaP.normalize();
  Ff = PVector.mult(deltaP, (leftTerm + rightTerm)); 
}

void setup() {
  size(800, 600);
  center = new PVector(width/2.0, height/2.0);
  particle = new Particle();
  particle.position = new PVector(center.x + radius*cos(angle), center.y + radius*sin(angle));
  particle.velocity = new PVector(0,0);
  particle.mass = 1;
  
  Fext = new PVector(0, particle.mass * 9.81);
  Fc = new PVector(0,0);
  Ff = new PVector(0,0);
}
 
void updateConstraintForce() {
  float lambda = 0; //Lagrange multiplier
  PVector X = particle.position.get();
  X = PVector.sub(X,center);
  float a = PVector.dot(Fext, X);
  float b = particle.mass*PVector.dot(particle.velocity, particle.velocity);
  float c = PVector.dot(X, X);
  lambda = (-a-b)/c;     
  Fc = PVector.mult(X, lambda);  ;
  System.out.println("Lambda: "+lambda);
}

void stepPhysics() {
  PVector force = Fext.get();
  updateConstraintForce();
  updateFeedbackForce();
  //System.out.println("Fc: ["+Fc.x+","+Fc.y+","+Fc.z+"]");
  force.add(Fc);
  force.add(Ff);
  
  PVector acceleration = new PVector(force.x / particle.mass, force.y / particle.mass);
  particle.velocity.x += acceleration.x * dt;
  particle.velocity.y += acceleration.y * dt;
  particle.position.x += particle.velocity.x * dt;
  particle.position.y += particle.velocity.y * dt;   
  
  //updateDistanceConstraint();
}

void draw() {
  background(200);
  
  stepPhysics();

  //draw line
  line(center.x, center.y, particle.position.x, particle.position.y);
  
  //draw path of particle
  noFill();
  ellipse(center.x, center.y, radius*2, radius*2);
 
  //draw particle
  fill(127); 
  ellipse(particle.position.x, particle.position.y, 10,10);
  
}
