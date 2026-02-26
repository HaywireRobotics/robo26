// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrappers;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.dylibso.chicory.runtime.ExportFunction;
import com.dylibso.chicory.runtime.HostFunction;
import com.dylibso.chicory.runtime.Instance;
import com.dylibso.chicory.runtime.Store;
import com.dylibso.chicory.wasi.WasiOptions;
import com.dylibso.chicory.wasi.WasiPreview1;
import com.dylibso.chicory.wasm.Parser;
import com.dylibso.chicory.wasm.WasmModule;
import com.dylibso.chicory.wasm.types.FunctionType;
import com.dylibso.chicory.wasm.types.ValType;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties.Value;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystem.ChooChooTrain;



record WasmFunction(
    Runnable init,
    Runnable exec,
    Consumer<Boolean> end,
    BooleanSupplier isDone,
    ArrayList<Dependencies> subsystems
) {}

enum Dependencies {
    ChooChooTrain
}

/** Add your docs here. */
public class WasmWrapper {
    static Instance instance;
    static HashMap<String, WasmFunction> wasmfunctions = new HashMap<>();

    static HostFunction wasmRegisterCommand = new HostFunction(
        "env",
        "register_command",
        FunctionType.of(
            List.of(
                ValType.I32, ValType.I32, // Name + length
                ValType.I32, ValType.I32, // Init fn + length
                ValType.I32, ValType.I32, // Exec fn + length
                ValType.I32, ValType.I32, // End fn + length
                ValType.I32, ValType.I32, // isDone fn + length
                ValType.I32 // Required Subsystems
            ),
            List.of()
        ), 
        (instance, args) -> {
            var name_ptr = (int) args[0];
            var name_length = (int) args[1];
            var init_ptr = (int) args[2];
            var init_length = (int) args[3];
            var execute_ptr = (int) args[4];
            var execute_length = (int) args[5];
            var end_ptr = (int) args[6];
            var end_length = (int) args[7];
            var isDone_ptr = (int) args[8];
            var isDone_length = (int) args[9];
            var subsystem_flags = (int) args[10];

            var name = instance.memory().readString(name_ptr, name_length);
            var init = instance.memory().readString(init_ptr, init_length);
            var execute = instance.memory().readString(execute_ptr, execute_length);
            var end = instance.memory().readString(end_ptr, end_length);
            var isDone = instance.memory().readString(isDone_ptr, isDone_length);

            System.out.printf("Registered WASM command: %s %s %s %s %s\n", name, init, execute, end, isDone);
            
            ArrayList<Dependencies> subsystems = new ArrayList<>();

            if ((subsystem_flags & 1) != 0) {
                subsystems.add(Dependencies.ChooChooTrain);
            }

            wasmfunctions.put(
                name, 
                new WasmFunction(
                    () -> {
                        instance.export(init).apply();
                    },
                    () -> {
                        instance.export(execute).apply();
                    },
                    (Boolean isCanceled) -> {
                        instance.export(end).apply();
                    },
                    () -> {
                        return instance.export(isDone).apply()[0] != 0;
                    },
                    subsystems
                )
            );

            return null;
        }
    );
    static HostFunction wasmDrive = new HostFunction(
        "env",
        "drivetrain_drive",
        FunctionType.of(
            List.of(
                ValType.F64, // x
                ValType.F64, // y
                ValType.F64 // rot
            ),
            List.of()
        ),
        (instance, args) -> {

            ChooChooTrain.instance.drive(
                (double) args[0],
                (double) args[1],
                (double) args[2],
                false
            );

            return null;
        }
    );


    static Store store = new Store();

    static ArrayList<Instance> instances = new ArrayList<>();

    public static void init() {
        store.addFunction(wasmRegisterCommand);
        store.addFunction(wasmDrive);
        //Silly Goose! Try to find all the errors the goose made when he flapped all across the screen!! :)
        var options = WasiOptions.builder().withStdout(System.out).build();
        // create our instance of wasip1
        var wasi = WasiPreview1.builder().withOptions(options).build();
        // create the module and connect the host functions
        store.addFunction(wasi.toHostFunctions());

        File directory = Filesystem.getDeployDirectory();
        File[] filesList = directory.listFiles();

        if (filesList != null) {
            for (File file : filesList) {
                if (file.isFile() && file.getName().endsWith(".wasm")) {
                    try {
                        System.out.printf("[WASM] Running file \"%s\"\n", file.getName());

                        WasmModule module = Parser.parse(file);
                        Instance instance = store.instantiate("CommandIntegration", module);
                        instances.add(instance);
                        ExportFunction main = instance.export("entry");
                        main.apply();
                    } catch (Exception exception) {
                        System.err.printf("File '%s' wasn't able to be executed:\n", file.getName());
                        System.err.println(exception);
                    }
                }
            }
        }
    }

    public static Command getCommand(String name) {
        System.out.printf("[WASM] Requested Command %s\n", name);
        if(
            wasmfunctions.containsKey(name)
        ){
            WasmFunction fn = wasmfunctions.get(name);
            ArrayList<Dependencies> dependencies = fn.subsystems();
            ArrayList<Subsystem> subsystems = new ArrayList<>();
            for (Dependencies dependency : dependencies) {
                if (dependency == Dependencies.ChooChooTrain) {
                    subsystems.add(ChooChooTrain.instance);
                }
            }
            return new FunctionalCommand(
                fn.init(),
                fn.exec(),
                fn.end(),
                fn.isDone(),
                subsystems.toArray(new Subsystem[0])
            );
        } else {
            return new PrintCommand("No Command Found");
        }
    }
}
