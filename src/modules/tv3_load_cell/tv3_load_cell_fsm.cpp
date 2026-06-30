#include "tv3_load_cell_fsm.hpp"

namespace tv3
{

void LoadCellFsm::apply_module_mode(const tv3_sm_modes_s &modes)
{
	switch (modes.load_cell_mode) {
	case tv3_sm_modes_s::LOAD_CELL_MONITOR:
		_fsm.request(LoadCellMode::Monitor);
		break;

	default:
		_fsm.request(LoadCellMode::Off);
		break;
	}

	_fsm.apply_request();
}

} // namespace tv3
