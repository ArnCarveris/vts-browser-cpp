#ifndef CACHE_H_seghioqnh
#define CACHE_H_seghioqnh

#include <string>

namespace melown
{
    class Cache
    {
    public:
        Cache(const class MapOptions &options);
        ~Cache();

        bool read(const std::string &name, void *&buffer, uint32 &size);
        void purge(const std::string &name);
        void purgeAll();
    };
}

#endif