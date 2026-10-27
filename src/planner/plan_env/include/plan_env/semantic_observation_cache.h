#ifndef PLAN_ENV_SEMANTIC_OBSERVATION_CACHE_H
#define PLAN_ENV_SEMANTIC_OBSERVATION_CACHE_H
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>

namespace apexnav_planner {
// Match delayed inference only to its original geometry. Integer nanoseconds
// avoid clock-type conversion and floating-point timestamp comparisons.
template<class Observation> class SemanticObservationCache {
public:
  struct Match { int64_t stamp; Observation observation; double score; };
  SemanticObservationCache(size_t capacity=600, int64_t max_age=15000000000LL,
                           int64_t epoch=0)
      : capacity_(capacity), max_age_(max_age), retired_(epoch) {
    if (!capacity || max_age <= 0) throw std::invalid_argument("Invalid semantic cache limits");
  }
  std::optional<Match> addObservation(int64_t stamp, int64_t now, Observation observation) {
    if (!accept(stamp, now)) return std::nullopt;
    auto& entry = entries_[stamp];
    if (!entry.observation) entry.observation = std::move(observation);
    return finish(stamp);
  }
  std::optional<Match> addScore(int64_t stamp, int64_t now, double score) {
    if (!std::isfinite(score) || score < -1.0 || score > 1.0 || !accept(stamp, now))
      return std::nullopt;
    auto& entry = entries_[stamp];
    if (!entry.score) entry.score = score;
    return finish(stamp);
  }
  std::optional<Observation> observation(int64_t stamp, int64_t now) {
    while (!entries_.empty() && now - entries_.begin()->first > max_age_) retireFirst();
    auto it = entries_.find(stamp);
    return it == entries_.end() ? std::nullopt : it->second.observation;
  }
  size_t size() const { return entries_.size(); }
  size_t dropped() const { return dropped_; }
private:
  struct Entry { std::optional<Observation> observation; std::optional<double> score; };
  std::map<int64_t, Entry> entries_;
  size_t capacity_, dropped_ = 0;
  int64_t max_age_, retired_;
  void retireFirst() {
    retired_ = std::max(retired_, entries_.begin()->first);
    if (entries_.begin()->second.score) ++dropped_;
    entries_.erase(entries_.begin());
  }
  bool accept(int64_t stamp, int64_t now) {
    while (!entries_.empty() && now - entries_.begin()->first > max_age_) retireFirst();
    if (stamp <= retired_ || stamp > now || now - stamp > max_age_) {
      ++dropped_;
      return false;
    }
    return true;
  }
  std::optional<Match> finish(int64_t stamp) {
    auto& entry = entries_.at(stamp);
    if (entry.observation && entry.score) {
      Match result{stamp, *entry.observation, *entry.score};
      entry.score.reset();
      // A single serial inference worker produces monotonic timestamps.
      // Discard late older results rather than reintroducing stale evidence.
      retired_ = stamp;
      entries_.erase(entries_.begin(), entries_.lower_bound(stamp));
      return result;
    }
    while (entries_.size() > capacity_) retireFirst();
    return std::nullopt;
  }
};
}  // namespace apexnav_planner
#endif
