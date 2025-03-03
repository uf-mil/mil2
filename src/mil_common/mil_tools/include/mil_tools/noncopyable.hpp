namespace mil_tools
{

/** Inherit from this class to prevent copying and assignment. */
class noncopyable
{
protected:
  noncopyable() = default;
  ~noncopyable() = default;

  noncopyable(noncopyable const&) = delete;
  noncopyable& operator=(noncopyable const&) = delete;

  noncopyable(noncopyable&&) = default;
  noncopyable& operator=(noncopyable&&) = default;
};

}  // namespace mil_tools
