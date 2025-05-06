if [ "$1" != "" ]; then
  echo "Auton debug mode ON"
  cargo v5 build --release --features no_selector
  echo "Auton debug mode ON"
  cargo v5 run --slot 2 --release --features no_selector
else
  echo "Auton debug mode OFF"
  cargo v5 upload --slot 1 --release
  echo "Auton debug mode OFF: final comp release uploaded to slot 1"
fi
