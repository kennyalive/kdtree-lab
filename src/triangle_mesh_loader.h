#pragma once

#include <memory>
#include <string>

struct TriangleMesh;
std::unique_ptr<TriangleMesh> LoadTriangleMesh(const std::string& fileName);
