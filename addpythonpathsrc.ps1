if (([string]::IsNullOrEmpty($ENV:PYTHONPATH)))
{
    $ENV:PYTHONPATH=$PSScriptRoot+"\deepracing_rclpy"
}
else
{
    $ENV:PYTHONPATH=$PSScriptRoot+"\deepracing_rclpy;"+$ENV:PYTHONPATH
}