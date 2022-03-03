#pragma once

namespace AVP_SIM
{
    struct SemanticInformation
	{
		float k;
		char type;
		float centrex;
		float centrey;
		float cornerx1;
		float cornery1;
		float cornerx2;
		float cornery2;
		int parkcode;
	};

	struct DottedLine
	{
		float x0, y0;
		float x1, y1;
	};

	struct StraightArrow
	{
		float x0, y0;
		float x1, y1;
		float x2, y2;
	};

	struct ArrowTurns
	{
		float x0, y0;
		float x1, y1;
		float x2, y2;
	};

	struct StraightTurningArrow
	{
		float x0, y0;
		float x1, y1;
		float x2, y2;
	};

	struct DoubleArrow
	{
		float x0, y0;
		float x1, y1;
		float x2, y2;
	};


	
}