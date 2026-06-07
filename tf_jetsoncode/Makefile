cross-build-debug:
	/home/eb/.cargo/bin/cross build --target aarch64-unknown-linux-gnu
cross-build-release:
	/home/eb/.cargo/bin/cross build --target aarch64-unknown-linux-gnu --release

upload-debug: cross-build-debug
	ssh robotik@10.0.64.101 "bash /home/robotik/tf_jetsoncode/kill.sh" && scp target/aarch64-unknown-linux-gnu/debug/tf_jetsoncode robotik@10.0.64.101:/home/robotik/Documents/teamfaabs-ssl-robot-code/tf_jetsoncode

upload-release: cross-build-release
	ssh robotik@10.0.64.102 "pkill tf_jetsoncode || true" && scp target/aarch64-unknown-linux-gnu/release/tf_jetsoncode robotik@10.0.64.102:/home/robotik/tf_jetsoncode


run-debug: upload-debug
	ssh robotik@10.0.64.101 "cd /home/robotik/tf_jetsoncode/tf_jetsoncode && ./tf_jetsoncode"

run-release: upload-release
	ssh robotik@10.0.64.102 "cd /home/robotik/tf_jetsoncode && ./tf_jetsoncode"
	#ssh robotik@10.0.64.101 "sudo systemctl restart tf_jetsoncode.service"