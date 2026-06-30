#pragma once

namespace tv3
{

template<typename Mode>
struct Tv3ModuleFsm {
	Mode mode{};
	Mode requested{};

	void request(Mode m) { requested = m; }

	void apply_request() { mode = requested; }

	bool in_mode(Mode m) const { return mode == m; }
};

} // namespace tv3
