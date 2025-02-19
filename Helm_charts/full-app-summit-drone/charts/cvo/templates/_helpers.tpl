{{- define "cvo.labels" -}}
app: {{ .Chart.Name }}
{{- end }}

{{- define "cvo.selectorLabels" -}}
app: {{ .Values.appName }}
{{- end }}

