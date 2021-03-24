# Solder_temperature_controller

## 1.0 Introduction
# 1.1 Project Description
This section will provide a brief overview of the development activity and system context as delineated in the following two subsections.


1.1.1 Background
Summarize the conditions that created the need for the new system. Provide the high-level mission goals and objective of the system operation. Provide the rationale for the development of the system.


1.1.2 Assumptions and Constraints
State the basic assumptions and constraints in the development of the concept. For example, that some technology will be matured enough by the time the system is ready to be fielded, or that the system has to be provided by a certain date in order to accomplish the mission.


# 1.2 Overview of the Envisioned System
This section provides an executive summary overview of the envisioned system. A more detailed description will be provided in Section 3.0

The goal of the project is to make a temperature controller for soldering iron powered with a PID controller for getting better, more precise results. 

1.2.1 Overview
This subsection provides a high-level overview of the system and its operation. Pictorials, graphics, videos, models, or other means may be used to provide this basic understanding of the concept.


1.2.2 System Scope
This section gives an estimate of the size and complexity of the system. It defines the system’s external interfaces and enabling systems. It describes what the project will encompass and what will lie outside of the project’s development.


## 2.0 Documents
# 2.1 Applicable Documents
This section lists all the documents, models, standards or other material that are applicable and some or all of which will form part of the requirements of the project.


# 2.2 Reference Documents
This section provides supplemental information that might be useful in understanding the system or its scenarios.


## 3.0 Description of Envisioned System
This section provides a more detailed description of the envisioned system and its operation as contained in the following subsections.

The system built should have a temperature sensor, a soldering iron, a controller, an LCD display, a switch and a device for input. When the system is switched on it is expected that it will receive an input temperature (desired value) from the user, have it on the LCD display and at the same time with the use of a sensor (connected to the tip of the soldering iron) it should get a value of the actual temperature. Both values should be compared and according to the difference the controller should make a decision of whether to turn the heating off to cool the temperature down or let it keep on to raise.  


# 3.1 Needs, Goals and Objectives of Envisioned System
This section describes the needs, goals, and objectives as expectations for the system capabilities, behavior, and operations. It may also point to a separate document or model that contains the current up-to-date agreed-to expectations.

 

# 3.2 Overview of System and Key Elements
This section describes at a functional level the various elements that will make up the system, including the users and operators. These descriptions should be implementation free; that is, not specific to any implementation or design but rather a general description of what the system and its elements will be expected to do. Graphics, pictorials, videos, and models may be used to aid this description.

Control systems are used to make control decisions depending on the desired state and the output state of a system. For controlling the temperature for the soldering iron we need a suitable control system. 

The purpose of feedback control is to keep the controlled variable close to its set point. This is achieved by using the difference between the set point and the controlled variable to determine the value of the input to the feedback controller.

For this project we rely on a feedback control system because after some time those make the system act more stable, immune to noise and overall less sensitive to variation in component values.


The PID controller is the most common control algorithm. 
Mathematically the PID controller can be written in the following way:
u=Kce+Kiedt+Kd(dedt)

In the following formula u is the control signal and e is the control error.  According to this the control signal is thus a sum of three components: proportional, integral and derivative. The controller parameters Kc, Ki, and Kd relative weights of components which can be interpreted as control actions based on the past, the present and the future.








# 3.3 Interfaces
This section describes the interfaces of the system with any other systems that are external to the project. It may also include high-level interfaces between the major envisioned elements of the system. Interfaces may include mechanical, electrical, human user/operator, fluid, radio frequency, data, or other types of interactions.


# 3.4 Modes of Operations
This section describes the various modes or configurations that the system may need in order to accomplish its intended purpose throughout its life cycle. This may include modes needed in the development of the system, such as for testing or training, as well as various modes that will be needed during it operational and disposal phases.


# 3.5 Proposed Capabilities
This section describes the various capabilities that the envisioned system will provide. These capabilities cover the entire life cycle of the system’s operation, including special capabilities needed for the verification/validation of the system, its capabilities during its intended operations, and any special capabilities needed during the decommissioning or disposal process.

The final system should have a high efficiency algorithm that will ensure very accurate results. 
# 4.0 Physical Environment
This section should describe the environment that the system will be expected to perform in throughout its life cycle, including integration, tests, and transportation. This may include expected and off-nominal temperatures, pressures, radiation, winds, and other atmospheric, space, or aquatic conditions. A description of whether the system needs to operate, tolerate with degraded performance, or just survive in these conditions should be noted.


# 5.0 Support Environment
This section describes how the envisioned system will be supported after being fielded. This includes how operational planning will be performed and how commanding or other uploads will be determined and provided, as required. Discussions may include how the envisioned system would be maintained, repaired, replaced, it’s sparing philosophy, and how future upgrades may be performed. It may also include assumptions on the level of continued support from the design teams.


# 6.0 Operational Scenarios, Use Cases and/or Design Reference Missions
This section takes key scenarios, use cases, or DRM and discusses what the envisioned system provides or how it functions throughout that single-thread timeline. The number of scenarios, use cases, or DRMs discussed should cover both nominal and off-nominal conditions and cover all expected functions and capabilities. A good practice is to label each of these scenarios to facilitate requirements traceability; e.g., [DRM-0100], [DRM-0200], etc.

6.1 Nominal Conditions
These scenarios, use cases, or DRMs cover how the envisioned system will operate under normal circumstances where there are no problems or anomalies taking place.


6.2 Off-Nominal Conditions
These scenarios cover cases where some condition has occurred that will need the system to perform in a way that is different from normal. This would cover failures, low performance, unexpected environmental conditions, or operator errors. These scenarios should reveal any additional capabilities or safeguards that are needed in the system.


# 7.0 Impact Considerations
This section describes the potential impacts, both positive and negative, on the environment and other areas.

# 7.1 Environmental Impacts
Describes how the envisioned system could impact the environment of the local area, state, country, worldwide, space, and other planetary bodies as appropriate for the systems intended purpose. This includes the possibility of the generation of any orbital debris, potential contamination of other planetary bodies or atmosphere, and generation of hazardous wastes that will need disposal on earth and other factors. Impacts should cover the entire life cycle of the system from development through disposal.


# 7.2 Organizational Impacts
Describes how the envisioned system could impact existing or future organizational aspects. This would include the need for hiring specialists or operators, specialized or widespread training or retraining, and use of multiple organizations.


# 7.3 Scientific/Technical Impacts
This subsection describes the anticipated scientific or technical impact of a successful mission or deployment, what scientific questions will be answered, what knowledge gaps will be filled, and what services will be provided. If the purpose of this system is to improve operations or logistics instead of science, describe the anticipated impact of the system in those terms.


# 8.0 Risks and Potential Issues
This section describes any risks and potential issues associated with the development, operations or disposal of the envisioned system. Also includes concerns/risks with the project schedule, staffing support, or implementation approach. Allocate subsections as needed for each risk or issue consideration. Pay special attention to closeout issues at the end of the project.


Appendix A: Acronyms
This part lists each acronym used in the ConOps and spells it out.


Appendix B: Glossary of Terms
The part lists key terms used in the ConOps and provides a description of their meaning.
