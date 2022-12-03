#pragma once

class oneField {
public:
	oneField();
	bool getMyEmp();
	void setMyEmp(bool myEmp);
private:
	inline static bool myEmp = false;
};