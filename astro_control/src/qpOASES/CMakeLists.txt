cmake_minimum_required(VERSION 2.8.3)

##############
## Building ##
##############

include_directories(include)

#
# building qpOASES LIBRARY
#
add_library(qpOASES
  src/BLASReplacement.cpp
  src/Bounds.cpp
  src/Constraints.cpp
  src/Flipper.cpp
  src/Indexlist.cpp
  src/LAPACKReplacement.cpp
  src/Matrices.cpp
  src/MessageHandling.cpp
  src/Options.cpp
  src/OQPinterface.cpp 
  src/QProblem.cpp
  src/QProblemB.cpp
  src/SolutionAnalysis.cpp
  src/SubjectTo.cpp
  src/SparseSolver.cpp
  src/SQProblem.cpp
  src/SQProblemSchur.cpp
  src/SubjectTo.cpp  
  src/Utils.cpp)

# #
# # building example applications
# #
set(EXAMPLES
  example1
  example1a
  example1b
  example2
  example3
  example3b
  example4
  example5
  exampleLP
  qrecipe)
 
foreach(EXAMPLE ${EXAMPLES})
  add_executable(${EXAMPLE} examples/${EXAMPLE}.cpp)
  target_link_libraries(${EXAMPLE} qpOASES)
endforeach(EXAMPLE)