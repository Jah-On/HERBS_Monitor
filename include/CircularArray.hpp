template <typename T, size_t S>
class CircularArray {
public:
  CircularArray() = default;

  void push(T value) noexcept {
    switch (used) {
    case S:
      members[start]  = value;
      ++start %= S;
      break;
    default:
      members[used++] = value;
      break;
    }
  }

  inline T at(size_t index) const noexcept {
    return this[index];
  }

  inline T& operator[] (size_t index) noexcept {
    return members[(index + start) % S];
  }

  inline size_t size() const noexcept {
    return used;
  }

  inline size_t capacity() const noexcept {
    return S;
  }
private:
  size_t start     = 0;
  size_t used      = 0;
  T      members[S];
};