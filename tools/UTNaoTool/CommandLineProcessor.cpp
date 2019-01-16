#include <tool/CommandLineProcessor.h>

#include <VisionCore.h>
#include <common/Profiling.h>
#include <memory/LogViewer.h>
#include <communications/CommunicationModule.h>
#include <tool/simulation/LocalizationSimulation.h>
#include <tool/simulation/BehaviorSimulation.h>
#include <tool/simulation/IsolatedBehaviorSimulation.h>

#include <thread>

using namespace std::literals;

int CommandLineProcessor::runLogServer(std::string source, bool loop) {
  auto core = std::make_unique<VisionCore>(CORE_TOOL,false,0,1);
  auto comm = std::make_unique<CommunicationModule>(core.get());
  auto log = std::make_unique<LogViewer>(source);
  auto memory = &log->getFrame(0);
  comm->init(memory, core->textlog_.get());
  comm->startTCPServer();
  do {
    for(int i = 0; i < log->size(); i++) {
      memory = &log->getFrame(i);
      comm->updateModuleMemory(memory);
      comm->optionallyStream();
      std::this_thread::sleep_for(500ms);
    }
  } while(loop);
  return 0;
}

int CommandLineProcessor::runBehaviorSim() {
  // 10 players. Enable localization to generate randomness in experiments
  auto simulation = std::make_unique<BehaviorSimulation>(true);

  while (!simulation->complete()) {
    simulation->simulationStep();
  }
  std::cout << simulation->simBlueScore << " " << simulation->simRedScore << std::endl;

  return 0;
}

int CommandLineProcessor::runLocalizationSim() {
  std::vector<AgentError> errors;
  for(int i = 0; i < 100; i++) {
    tic();
    auto seed = rand();
    auto path = SimulationPath::generate(10, seed);
    auto psim = new LocalizationSimulation(LocSimAgent::Type::Default);
    fprintf(stderr, "Running simulation with seed %i\n", seed);
    psim->setPath(path);
    auto func = [] (LocalizationSimulation* sim) {
      while(!sim->complete()) {
        sim->simulationStep();
      }
    };
    auto pthread = std::thread(func, psim);
    pthread.join();
    fprintf(stderr, "Sim time: %2.2f seconds\n", toc());
    psim->printError();
    errors.push_back(psim->getError(LocSimAgent::Type::Default));
    auto avg = AgentError::average(errors);
    fprintf(stderr, "Avg dist: %2.2f, Avg rot: %2.2f, Avg steps: %2.2f\n",
      avg.dist, avg.rot, avg.steps
    );
    fprintf(stderr, "----------------------------------------------------------\n");
  }
  return 0;
}

int CommandLineProcessor::runPlanningSim() {
  std::vector<std::vector<WorldObjectType>> obs = {
    { // Config 1
      WO_OBSTACLE_UNKNOWN_3,
      WO_OBSTACLE_UNKNOWN_4,
      WO_OBSTACLE_UNKNOWN_5, 
      WO_OBSTACLE_UNKNOWN_6, 
      WO_OBSTACLE_UNKNOWN_12,
      WO_OBSTACLE_UNKNOWN_15,
    }, 
    { // Config 2
      WO_OBSTACLE_UNKNOWN_1,
      WO_OBSTACLE_UNKNOWN_2,
      WO_OBSTACLE_UNKNOWN_3,
      WO_OBSTACLE_UNKNOWN_4,
      WO_OBSTACLE_UNKNOWN_9, 
      WO_OBSTACLE_UNKNOWN_11,
    },
    { // Config 3
      WO_OBSTACLE_UNKNOWN_8, 
      WO_OBSTACLE_UNKNOWN_9, 
    },
    { // Config 4
      WO_OBSTACLE_UNKNOWN_5, 
      WO_OBSTACLE_UNKNOWN_6, 
      WO_OBSTACLE_UNKNOWN_7, 
      WO_OBSTACLE_UNKNOWN_9, 
      WO_OBSTACLE_UNKNOWN_11,
    },
    { // Config 5
      WO_OBSTACLE_UNKNOWN_10,
      WO_OBSTACLE_UNKNOWN_12,
      WO_OBSTACLE_UNKNOWN_15,
    },
    { // Config 6
      WO_OBSTACLE_UNKNOWN_1,
      WO_OBSTACLE_UNKNOWN_2,
      WO_OBSTACLE_UNKNOWN_4,
      WO_OBSTACLE_UNKNOWN_7, 
    },
    { // Config 7
      WO_OBSTACLE_UNKNOWN_8, 
      WO_OBSTACLE_UNKNOWN_9, 
    },
    { // Config 8
      WO_OBSTACLE_UNKNOWN_5, 
      WO_OBSTACLE_UNKNOWN_7, 
      WO_OBSTACLE_UNKNOWN_9, 
      WO_OBSTACLE_UNKNOWN_11,
      WO_OBSTACLE_UNKNOWN_13,
    },
    { // Config 9
      WO_OBSTACLE_UNKNOWN_3,
      WO_OBSTACLE_UNKNOWN_6, 
    },
  };

  for (int i = 0; i < obs.size(); i++) {
    fprintf(stderr, "Config %d\n", i + 1);
    
    // AStar
    auto Asimulation = std::make_unique<IsolatedBehaviorSimulation>(&(obs.at(i)), true, false);
    
    tic();
    while (!Asimulation->complete()) {
      Asimulation->simulationStep();
    }
    
    fprintf(stderr, "A* Sim time: %2.2f seconds\n", toc());
    
    fprintf(stderr, "nodeExp: %d  pathsPlanned: %d nodesInPath: %d\n", Asimulation->sim_.cache_.planning->nodeExpansions,
            Asimulation->sim_.cache_.planning->pathsPlanned, Asimulation->sim_.cache_.planning->nodesInPath);

    // DStar
    auto Dsimulation = std::make_unique<IsolatedBehaviorSimulation>(&(obs.at(i)), false, false);
    tic();
    while (!Dsimulation->complete()) {
      Dsimulation->simulationStep();
    }
    fprintf(stderr, "D* Lite Sim time: %2.2f seconds\n", toc());
    

    fprintf(stderr, "nodeExp: %d  pathsPlanned: %d nodesInPath: %d\n", Dsimulation->sim_.cache_.planning->nodeExpansions,
            Dsimulation->sim_.cache_.planning->pathsPlanned, Dsimulation->sim_.cache_.planning->nodesInPath);

    fprintf(stderr, "----------------------------------------------------------\n");

    if (i == 0) i = 5;
  }  
  return 0;
}
