

#include "../common/glUtils.h"

class Snapshot {

public:

  Snapshot(char const * filename);

  ~Snapshot();

private:
  
  struct glSnapshot;

  glSnapshot * _glSnapshot;
};
