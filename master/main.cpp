#include <chrono>
#include <iostream>
#include <thread>
#include <filesystem>
#include <csignal>
#include <unistd.h>
#include <wiringPi.h>
using namespace std::chrono_literals;
int main()
{
	std::error_code err;
	std::filesystem::remove("/home/pi/map1", err);
	std::filesystem::remove("/home/pi/map2", err);
	constexpr auto BUTTON_PIN = 4;
	wiringPiSetup();
	pinMode(BUTTON_PIN, INPUT);
	pid_t pid = 0;
	while (true)
	{
		if (digitalRead(BUTTON_PIN) == 0)
		{
			if (pid)
			{
				kill(pid, SIGKILL);
				pid = 0;
				std::cout << "kill" << std::endl;
				std::this_thread::sleep_for(2s);
				continue;
			}
			pid = fork();
			if (pid == -1)
			{
				std::perror("fork");
				return 1;
			}
			if (!pid)
			{
				if (execl("/home/pi/RCJ-Rescue-Maze-2023/pi/build/maze-2023", "maze-2023", NULL) == -1)
				{
					std::perror("execl");
					std::exit(1);
				}
			}
			std::this_thread::sleep_for(1s);
		}
		std::this_thread::sleep_for(10ms);
	}
}
