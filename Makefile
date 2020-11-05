bin/robot: obj/main.o obj/BodyMotion.o obj/RobotBuild.o obj/Kinematics.o
	g++ -o bin/robot -g obj/main.o obj/BodyMotion.o obj/RobotBuild.o obj/Kinematics.o

obj/main.o: src/main.cpp src/BodyMotion.hpp src/RobotBuild.hpp
	g++ -c -g src/main.cpp -o obj/main.o

obj/BodyMotion.o: src/BodyMotion.cpp src/BodyMotion.hpp
	g++ -c -g src/BodyMotion.cpp -o obj/BodyMotion.o

obj/RobotBuild.o: src/RobotBuild.cpp src/BodyMotion.hpp src/RobotBuild.hpp
	g++ -c -g src/RobotBuild.cpp -o obj/RobotBuild.o

obj/Kinematics.o: src/RobotBuild.cpp src/BodyMotion.hpp src/RobotBuild.hpp src/Kinematics.cpp src/Kinematics.hpp
	g++ -c -g src/Kinematics.cpp -o obj/Kinematics.o
clean:
	rm -f obj/*.o bin/robot
