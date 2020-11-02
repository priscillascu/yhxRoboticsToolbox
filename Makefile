bin/robot: obj/main.o obj/BodyMotion.o obj/RobotBuild.o
	g++ -o bin/robot -g obj/main.o obj/BodyMotion.o obj/RobotBuild.o 

obj/main.o: src/main.cpp src/BodyMotion.hpp src/RobotBuild.hpp
	g++ -c -g src/main.cpp -o obj/main.o

obj/BodyMotion.o: src/BodyMotion.cpp src/BodyMotion.hpp
	g++ -c -g src/BodyMotion.cpp -o obj/BodyMotion.o

obj/RobotBuild.o: src/RobotBuild.cpp src/BodyMotion.hpp src/RobotBuild.hpp
	g++ -c -g src/RobotBuild.cpp -o obj/RobotBuild.o

clean:
	rm -f obj/*.o bin/robot
