#include "search.hpp"
#include "camera.hpp"
#include <thread>
#include <mutex>
#include <array>
#include <condition_variable>
#include <filesystem>

int main(int argc, char **argv) {
	std::atomic<ThreadState> thread_state = ThreadState::INIT;
	std::mutex map_lock;
	std::condition_variable map_cv;

	std::string port;
	for (const auto &entry: std::filesystem::directory_iterator("/sys/class/tty")) {
		const auto &filename = entry.path().filename();
		if (filename.generic_string().rfind("ttyACM", 0) == 0) {
			port = "/dev/" / filename;
		}
	}


	Serial serial(port, 9600);

	//Search search = (argc == 1) ? Search(map_lock, map_cv) : Search(argv[1],map_lock,map_cv);
	std::array<Search, 2> searches{
			(!std::filesystem::exists("/home/pi/map1")) ? Search(serial,map_lock, map_cv, "/home/pi/map1") :
			Search(serial,"/home/pi/map1", map_lock, map_cv),
			(!std::filesystem::exists("/home/pi/map2")) ? Search(serial,map_lock, map_cv, "/home/pi/map1") :
			Search(serial,"/home/pi/map2", map_lock, map_cv)};

	Search *search = &searches[0];
	std::uint8_t current = 0;

	std::thread camera_thread(&detect, std::ref(thread_state), &search, std::ref(map_lock), std::ref(map_cv));

	//search.print_map();
	std::stack<std::uint8_t> path;
	//initial wall check
	search->check_walls();

	//wait for setup to complete
	while (thread_state == ThreadState::INIT) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	restart:

	//while unmarked tiles exist
	while (!(path = search->search()).empty()) {
		if (!search->move(path)) {
			current = !current;
			map_lock.lock();
			search = &searches[current];
			search->map[search->y][search->x][Dir::S] = true;
			map_lock.lock();
		}
		search->check_walls();
		search->print_map();
	}

	//set starting point as unmarked and return to start

	if (search == &searches[1]) {
		map_lock.lock();
		search->map[search->y][search->x][Dir::S] = false;
		search->unmark_start();
		map_lock.unlock();
		goto restart;
	}

	map_lock.lock();
	search->unmark_start();
	map_lock.unlock();
	search->move(search->search());

	thread_state = ThreadState::STOP;
	camera_thread.join();
}
