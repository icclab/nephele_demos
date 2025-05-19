{{/*
Expand the name of the chart.
*/}}
{{- define "flask-gui.name" -}}
{{ .Chart.Name }}
{{- end }}

{{/*
Create a default fully qualified app name.
*/}}
{{- define "flask-gui.fullname" -}}
{{ .Release.Name }}-{{ include "flask-gui.name" . }}
{{- end }}

{{/*
Common labels for all resources.
*/}}
{{- define "flask-gui.labels" -}}
app: {{ include "flask-gui.name" . }}
chart: {{ .Chart.Name }}-{{ .Chart.Version }}
release: {{ .Release.Name }}
heritage: {{ .Release.Service }}
{{- end }}

{{/*
Selector labels for Deployment and Service.
*/}}
{{- define "flask-gui.selectorLabels" -}}
app: {{ include "flask-gui.name" . }}
release: {{ .Release.Name }}
{{- end }}

