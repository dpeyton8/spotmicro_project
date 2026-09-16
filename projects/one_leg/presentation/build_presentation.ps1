param(
    [string]$Template = 'C:\Users\David\Downloads\SpotMicro_LC_Template_Final_v2.pptx',
    [string]$Output = (Join-Path (Split-Path $PSScriptRoot -Parent) 'SpotMicro_One_Leg_Controls.pptx')
)

$ErrorActionPreference = 'Stop'
$work = Join-Path $env:TEMP ('spotmicro_one_leg_ppt_' + [guid]::NewGuid().ToString('N'))
$zip = Join-Path $work 'template.zip'
$expanded = Join-Path $work 'expanded'

New-Item -ItemType Directory -Path $work | Out-Null
Copy-Item -LiteralPath $Template -Destination $zip
Expand-Archive -LiteralPath $zip -DestinationPath $expanded

function Set-ShapeText {
    param([int]$Slide, [int]$ShapeId, [string[]]$Paragraphs)
    $path = Join-Path $expanded "ppt\slides\slide$Slide.xml"
    [xml]$xml = Get-Content -LiteralPath $path -Raw
    $ns = New-Object System.Xml.XmlNamespaceManager($xml.NameTable)
    $ns.AddNamespace('p', 'http://schemas.openxmlformats.org/presentationml/2006/main')
    $ns.AddNamespace('a', 'http://schemas.openxmlformats.org/drawingml/2006/main')
    $shape = $xml.SelectSingleNode("//p:sp[p:nvSpPr/p:cNvPr[@id='$ShapeId']]", $ns)
    if (-not $shape) { throw "Shape $ShapeId not found on slide $Slide" }
    $existing = @($shape.SelectNodes('./p:txBody/a:p', $ns))
    if ($existing.Count -eq 0) { throw "Shape $ShapeId on slide $Slide has no text paragraphs" }
    while ($existing.Count -lt $Paragraphs.Count) {
        $clone = $existing[$existing.Count - 1].CloneNode($true)
        [void]$shape.SelectSingleNode('./p:txBody', $ns).AppendChild($clone)
        $existing = @($shape.SelectNodes('./p:txBody/a:p', $ns))
    }
    for ($i = 0; $i -lt $existing.Count; $i++) {
        $nodes = @($existing[$i].SelectNodes('.//a:t', $ns))
        if ($nodes.Count -gt 0) {
            $nodes[0].InnerText = if ($i -lt $Paragraphs.Count) { $Paragraphs[$i] } else { '' }
            for ($j = 1; $j -lt $nodes.Count; $j++) { $nodes[$j].InnerText = '' }
        }
    }
    $xml.Save($path)
}

# 1 - title
Set-ShapeText 1 55 @('SpotMicro One-Leg Controls Testbench')
Set-ShapeText 1 56 @('David Peyton, Alanah Martinez, Seth Morales', 'University of New Mexico - Learning and Control Lab', 'Windows IK preview -> MuJoCo/RViz -> physical right leg')

# 2 - project framing
Set-ShapeText 2 63 @('Project Goal: One Right Leg, End to End')
Set-ShapeText 2 65 @('Why begin with one leg')
Set-ShapeText 2 66 @('- Model one physical right leg with three actuated joints', '- Command the toe in Cartesian space instead of guessing servo angles', '- Lower the toe to the ground, then reject small position disturbances', '- Reuse one interface from Windows preview through simulation and hardware')
Set-ShapeText 2 67 @('The experiment')
Set-ShapeText 2 68 @('Start above the floor; use IK/control to reach and hold the ground target.')
Set-ShapeText 2 69 @('Input')
Set-ShapeText 2 70 @('toe target')
Set-ShapeText 2 71 @('Model')
Set-ShapeText 2 72 @('right leg')
Set-ShapeText 2 73 @('Control')
Set-ShapeText 2 74 @('DLS Jacobian')
Set-ShapeText 2 75 @('Observe')
Set-ShapeText 2 76 @('toe error')
Set-ShapeText 2 77 @('Output')
Set-ShapeText 2 78 @('q1 / q2 / q3')
Set-ShapeText 2 79 @('3 joint targets')

# 3 - leg model
Set-ShapeText 3 85 @('One Leg = A Three-DOF Kinematic Chain')
Set-ShapeText 3 87 @('Desired toe', '(x, y, z)')
Set-ShapeText 3 89 @('Joint targets', 'q1, q2, q3')
Set-ShapeText 3 90 @('Forward kinematics: joint angles -> toe position', 'Inverse kinematics: toe target -> joint angles', 'Link lengths L1, L2, and L3 define reachability.', 'The right-leg knee branch keeps the leg bending in the intended direction.')
Set-ShapeText 3 91 @('Right leg: 3 DOF')
Set-ShapeText 3 93 @('q1')
Set-ShapeText 3 95 @('q2')
Set-ShapeText 3 97 @('q3')
Set-ShapeText 3 98 @('The same joint names are contracted across the testbench, MuJoCo, RViz, and hardware.')

# 4 - source-paper lessons
Set-ShapeText 4 108 @('What the IK Paper Contributes')
Set-ShapeText 4 109 @('Defines the single-leg frames and link geometry', 'Shows analytic inverse-kinematics relationships', 'Reveals two knee branches through the plus/minus square-root term', 'Defines reachability: impossible toe targets must be rejected', 'Highlights singular fully extended or folded poses', 'Important finding: the paper''s printed q1 equation is inconsistent', 'The repository-corrected IK is the analytic reference', 'The first feedback experiment uses a damped Jacobian to avoid fragile inversion')
Set-ShapeText 4 114 @('Use the paper for geometry and validation; do not copy the incorrect printed q1 formula into the controller.')

# 6 - controller comparison
Set-ShapeText 6 126 @('Controller Strategy and Validation')
Set-ShapeText 6 128 @('Analytic reference')
Set-ShapeText 6 130 @('Same right-leg geometry')
Set-ShapeText 6 132 @('Feedback experiment')
Set-ShapeText 6 133 @('- Corrected IK')
Set-ShapeText 6 134 @('Direct solution for a reachable toe target')
Set-ShapeText 6 135 @('- Damped Jacobian')
Set-ShapeText 6 136 @('Small joint updates reduce measured toe error')
Set-ShapeText 6 137 @('- Knee branch')
Set-ShapeText 6 138 @('Choose the right-leg positive-knee solution')
Set-ShapeText 6 139 @('- Safety checks')
Set-ShapeText 6 140 @('Clamp limits; reject unreachable and singular targets')
Set-ShapeText 6 141 @('- Compare results')
Set-ShapeText 6 142 @('Error, settling time, overshoot, and joint motion')
Set-ShapeText 6 143 @('Planned comparison: damped Jacobian vs repository-corrected analytic IK vs the paper''s printed equation.')

# 7 - control loop
Set-ShapeText 7 148 @('Toe-Hold Control Loop')
Set-ShapeText 7 149 @('Toe error becomes bounded joint updates at every tick.')
Set-ShapeText 7 150 @('Every control tick')
Set-ShapeText 7 151 @('1')
Set-ShapeText 7 152 @('Measure current toe position')
Set-ShapeText 7 153 @('2')
Set-ShapeText 7 154 @('Compute target - measured error')
Set-ShapeText 7 155 @('3')
Set-ShapeText 7 156 @('Apply damped Jacobian update')
Set-ShapeText 7 157 @('4')
Set-ShapeText 7 158 @('Command q1 / q2 / q3 and repeat')

# 8 - current Windows testbench (image12 is replaced below)
Set-ShapeText 8 165 @('Current Windows Testbench')
Set-ShapeText 8 167 @('Interactive sliders expose joint angles, toe targets, gain, and damping.', 'Run: lower the toe from its initial position to the ground, then hold it there.', 'This preview validates kinematics and control logic only - no gravity, torque, contact, or force model.')

# 9 - reserved slide
Set-ShapeText 9 172 @('Future VM Results: MuJoCo + RViz')
Set-ShapeText 9 173 @('RESERVED FOR MUJOCO + RVIZ SCREENSHOTS')
Set-ShapeText 9 174 @('Planned evidence:', '- toe target versus simulated toe position', '- joint-state and TF agreement in RViz', '- ground-contact response under gravity', '- settling time, overshoot, and steady-state error')

# 10 - hardware test bench
Set-ShapeText 10 182 @('Physical One-Leg Test Bench')
Set-ShapeText 10 183 @('Command path')
Set-ShapeText 10 184 @('PC test')
Set-ShapeText 10 186 @('ESP32')
Set-ShapeText 10 188 @('PCA9685')
Set-ShapeText 10 190 @('3 servos')
Set-ShapeText 10 191 @('Bench components')
Set-ShapeText 10 192 @('- Printed right-leg links and a rigid fixture', '- ESP32 for commands and test sequencing', '- PCA9685 16-channel PWM driver', '- Three MG996R servos now; JX PDI-HV5523MG later', '- Separate high-current servo power with a shared signal ground')
Set-ShapeText 10 193 @('SAFETY FIRST')
Set-ShapeText 10 194 @('Current limit', 'Joint limits', 'Emergency stop')
Set-ShapeText 10 195 @('Simulate -> calibrate -> test one joint -> close the loop')

# 11 - next work
Set-ShapeText 11 200 @('Next Work')
Set-ShapeText 11 201 @('1. Refine Windows testbench', 'Keep the HTML visualization and controls easy to demonstrate.')
Set-ShapeText 11 202 @('2. Complete Ubuntu VM', 'Install ROS2 Humble and the MuJoCo dependencies.')
Set-ShapeText 11 203 @('3. Run the integration contract', 'Match front_right_shoulder, front_right_leg, and front_right_foot.')
Set-ShapeText 11 204 @('4. Measure simulated control', 'Add contact, gravity, disturbances, and repeatable metrics.')
Set-ShapeText 11 205 @('5. Move to hardware', 'Calibrate servo centers and limits before loading the leg.')
Set-ShapeText 11 206 @('End goal: one controller concept that can be explained, simulated, measured, and safely tested on the physical right leg.')

$testbenchImage = Join-Path (Split-Path $PSScriptRoot -Parent) 'outputs\windows_testbench_after_lowering.png'
if (Test-Path -LiteralPath $testbenchImage) {
    Copy-Item -LiteralPath $testbenchImage -Destination (Join-Path $expanded 'ppt\media\image12.png') -Force
}

$outDir = Split-Path -Parent $Output
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$outZip = Join-Path $work 'output.zip'
Compress-Archive -Path (Join-Path $expanded '*') -DestinationPath $outZip
Copy-Item -LiteralPath $outZip -Destination $Output -Force
Remove-Item -LiteralPath $work -Recurse -Force
Write-Output $Output
