template <typename T, size_t S>
class RotaryArray {
private:
  T      members[S];
  size_t start     = 0;
  size_t reserved  = S;
  size_t used      = 0;
public:
  RotaryArray() = default;

  void push(T value){
    switch (used) {
    case S:
      members[start] = value;
      start += 1;
      start %= S;
      break;
    default:
      members[used]          = value;
      used++;
      break;
    }
  }

  T at(size_t index) const {
    return members[(index + start) % reserved];
  }

  T& operator[] (size_t index) {
    return members[(index + start) % reserved];
  }

  size_t size() const {
    return used;
  }

  size_t capacity() const {
    return reserved;
  }
};