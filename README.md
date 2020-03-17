# Particle-Simulation

PARTICLE SIMULATOR WRITTEN USING OPENGL

ABOUT:
 This is a Particle simulator written using C++ and ModernOpenGL. GLFW is used for Windowing functions. GLM is used for mathematical operations on vectors and matrices. 

PROGRAM STUCTURE:
 Each particle is represented as a structure having its own position, velocity, acceleration, color, birth time, age and active boolean.
 Euler's numerical integration method is used to calculate the particle position and velocity.
 All the active particles will be stored in an array of particle structures.
 An inactive particle list is defined as an integer stack and it stores the indices of all the inactive particles.
 A Collision detection and response system is setup to handle point-triangle collisions. 
 Uniform and Random distribution functions are defined to create both scalars and vectors. 
 The particles derive their attribute values from these distributions.
 The particles are rendered as points which can be easy rendered as spheres or other object by using a different VertexBuffer 

SIMULATION ALGORITHM: 
 Several particles are generated every frame until the maximum number of particles is reached. 
 As the particles generate, they are simulated to act according to the environmental accelerations like gravity, wind, etc
 Each particle has a random age and once they reach their age limit, they die and make space for a new particle to be generated
 As particles move in the 3D space, they check for collisions and act accordingly when they collide. 

HOW TO RUN:
 The entire project is uploaded as a Visual Studio Solution with all dependencies inside the project folder.
Download the Project and open the AnanthaNatarajan_ParticleSimulation.sln in Visual Studio. The program is set up for 32 bit environment
Once opened and successfully built, hit "S" key to start the siulation. 

TESTING:
 The macro "MaxParticles" determines the maximum number of particles that can be generated.
 The GenerateRandomParticle function sets the initial attributes of the particles. 
 The testAndDeactivate function looks for particles those are dead and deactivates it. 
 The findDeactiveSpot looks pops up the first element in the stack and gives to generateRandomParticle function. 
 As a simple case, the particles are generated towards a triangle uniformly distributed around the normal of the traingle with a max displacement of 30 degrees.
 Replacing the vectorGenerator_Du() or vectorGenerator_Dg() with vectorGenerator_S() in the statement that assigns a initial velocity to the particle will result in a particle generator generator particles in all directions
