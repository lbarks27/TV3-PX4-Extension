#pragma once

#include "../../lib/tv3_module_fsm.hpp"
#include "../../lib/tv3_module_modes.hpp"

#include <uORB/topics/tv3_sm_modes.h>

namespace tv3
{

class LoadCellFsm {
public:
	void apply_module_mode(const tv3_sm_modes_s &modes);

	bool in_mode(LoadCellMode mode) const { return _fsm.in_mode(mode); }

private:
	Tv3ModuleFsm<LoadCellMode> _fsm{LoadCellMode::Off};
};

} // namespace tv3
