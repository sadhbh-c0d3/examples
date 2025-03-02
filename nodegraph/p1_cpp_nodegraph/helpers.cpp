#include <fstream>
#include <iostream>
#include <thread>
#include <functional>

std::fstream debug_out;
std::mutex debug_out_mutex;

bool init_debugout(char const *path = "debug.log") {
    debug_out.open(path, std::ios::out | std::ios::trunc);
    if (!debug_out.good()) {
        std::cerr << "ERR> Cannot open debug file: " << path << std::endl;
        return false;
    }
    return true;
}

const char* animal_names[] = { "Badger", "Rabbit", "Weasel", "Monkey", "Beaver", "Jaguar", "Lemurs", "Walrus", "Ocelot", "Marten" };
const char* plant_names[] = { "Tulips", "Maples", "Lilacs", "Barley", "Grapes", "Bamboo", "Cactus", "Clover", "Melons", "Squash" };

std::string name_of_thread(std::thread::id id)
{
    std::hash<std::thread::id> hasher;
    size_t value = hasher(id);

    auto animal_name = animal_names[value % (sizeof animal_names/sizeof(char*))];

    return std::to_string(((value >> 11) % 29) + 11) + "_" + animal_name;
}
    
std::string name_of_entity(void *id)
{
    std::hash<void*> hasher;
    size_t value = hasher(id);

    auto plant_name = plant_names[value % (sizeof plant_names/sizeof(char*))];

    return std::to_string(((value >> 11) % 29) + 11) + "_" + plant_name;
}