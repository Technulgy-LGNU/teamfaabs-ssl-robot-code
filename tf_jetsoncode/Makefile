cross-build-debug:
	/home/eb/.cargo/bin/cross build --target aarch64-unknown-linux-gnu
cross-build-release:
	/home/eb/.cargo/bin/cross build --target aarch64-unknown-linux-gnu --release

upload-debug: cross-build-debug
	scp target/aarch64-unknown-linux-gnu/debug/tf_jetsoncode robotik@10.0.64.101:/home/robotik/Documents/teamfaabs-ssl-robot-code/tf_jetsoncode

upload-release: cross-build-release
	scp target/aarch64-unknown-linux-gnu/release/tf_jetsoncode robotik@10.0.64.101:/home/robotik/tf_jetsoncode


run-debug: upload-debug
	ssh robotik@10.0.64.101 "cd /home/robotik/tf_jetsoncode/tf_jetsoncode && ./tf_jetsoncode"

run-release: upload-release
	ssh robotik@10.0.64.101 "cd /home/robotik/tf_jetsoncode && ./tf_jetsoncode"
	#ssh robotik@10.0.64.101 "sudo systemctl restart tf_jetsoncode.service"