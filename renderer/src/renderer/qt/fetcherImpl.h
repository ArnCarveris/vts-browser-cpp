#ifndef FETCHERIMPL_H_djghfubhj
#define FETCHERIMPL_H_djghfubhj

#include <renderer/fetcher.h>

namespace
{
    class FetcherDetail;
}

class FetcherOptions
{
public:
    std::string username;
    std::string password;
};

class FetcherImpl : public melown::Fetcher
{
public:
    FetcherImpl(const FetcherOptions &options);
    ~FetcherImpl();

    void setOptions(const FetcherOptions &options);
    void setCallback(Func func) override;
    void fetch(melown::FetchType type, const std::string &name) override;

    FetcherDetail *impl;
};

#endif