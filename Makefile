file_finder = find . -type f $(1) -not -path './venv/*'

CMAKE_FILES = $(call file_finder,-name "*.cmake" -o -name "CMakeLists.txt")
PY_FILES = $(call file_finder,-name "*.py")
CPP_FILES = $(call file_finder,-regex '.*\.\(cpp\|hpp\|cu\|c\|h\)')
SH_FILES = $(call file_finder,-name "*.sh")

check: check_format check_sh_format pylint shellcheck clang_tidy

format:
	$(PY_FILES) | xargs black
	$(CMAKE_FILES) | xargs cmake-format -i
	$(CPP_FILES) | xargs clang-format --style=file -i
	shfmt -l -w .

check_format:
	$(PY_FILES) | xargs black --diff --check
	$(CMAKE_FILES) | xargs cmake-format --check

pylint:
	$(PY_FILES) | xargs pylint --rcfile=.pylintrc

check_sh_format:
	shfmt -d .

shellcheck:
	$(SH_FILES) | xargs shellcheck

clang_tidy:
	catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_CLANG_TIDY="clang-tidy-10"
	catkin build
